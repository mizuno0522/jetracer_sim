// 画面表示 (追従視点など) の後処理。車載カメラ (配信するセンサ画像) には掛けない。
// Built-in の描画で HDRP 版と同じ仕様の絵を出すためのもの (-quality medium 以上。数値は RenderCompat.GetLook が 3 版に同じ値を渡す):
//   物の際の陰り (AO) → 遠くのぼけ → 明るい所のにじみ (HDRP と同じく、ぼかした絵と混ぜる) → 周辺減光
//   → コントラスト・彩度 (HDRP の ColorAdjustments と同じ式) → ACES のトーンカーブ
// 入力は Linear・HDR (カメラの allowHDR)。URP 版・HDRP 版では使わない (それぞれの Volume が同じことをする)
Shader "Minicar/ViewPost"
{
    Properties
    {
        _MainTex ("Source", 2D) = "white" {}
        _Bloom ("Bloom", 2D) = "black" {}
        _Bloom2 ("Bloom wide", 2D) = "black" {}
        _Far ("Far blur", 2D) = "black" {}
        _AO ("AO", 2D) = "white" {}
    }
    SubShader
    {
        Cull Off ZWrite Off ZTest Always
        CGINCLUDE
        #include "UnityCG.cginc"
        sampler2D _MainTex, _Bloom, _Bloom2, _Far, _AO;
        UNITY_DECLARE_DEPTH_TEXTURE(_CameraDepthTexture);
        float4 _MainTex_TexelSize;
        float4 _Dir;        // ぼかしの向き (テクセル)
        float4 _Params;     // x = にじみの強さ, y = 露出 (倍率), z = コントラスト (1 + v/100), w = 彩度 (1 + v/100)
        float4 _Params2;    // x = 周辺減光の強さ, y = 周辺減光のなめらかさ, z = AO の強さ, w = AO の半径 [m]
        float4 _Dof;        // x = ぼけ始め [m], y = ぼけきる距離 [m], z = 有効 (0/1), w = トーンカーブ (0 = なし・1 = ACES)
        float4 _Proj;       // x = 1/P00, y = 1/P11, z = P00, w = P11 (視点空間 ↔ 画面)

        float EyeDepth(float2 uv) { return LinearEyeDepth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, uv)); }
        float3 ViewPos(float2 uv, float d) { return float3((uv * 2.0 - 1.0) * _Proj.xy * d, d); }

        // 物の際の陰り: 深度だけから。画素の位置から半球の向きに 8 点を取り、手前に物があれば陰らせる
        half4 fragAO(v2f_img i) : SV_Target
        {
            float d = EyeDepth(i.uv);
            if (d > _ProjectionParams.z * 0.9) return 1;
            float3 P = ViewPos(i.uv, d);
            float2 t = _MainTex_TexelSize.xy;
            // 法線: 左右・上下のうち深度の近い側の差分から (輪郭でのにじみを減らす)
            float dl = EyeDepth(i.uv - float2(t.x, 0)), dr = EyeDepth(i.uv + float2(t.x, 0));
            float du = EyeDepth(i.uv + float2(0, t.y)), dd = EyeDepth(i.uv - float2(0, t.y));
            float3 dx = abs(dl - d) < abs(dr - d) ? P - ViewPos(i.uv - float2(t.x, 0), dl) : ViewPos(i.uv + float2(t.x, 0), dr) - P;
            float3 dy = abs(dd - d) < abs(du - d) ? P - ViewPos(i.uv - float2(0, t.y), dd) : ViewPos(i.uv + float2(0, t.y), du) - P;
            float3 N = normalize(cross(dy, dx));
            float r = _Params2.w;
            float ang = frac(sin(dot(i.uv * _ScreenParams.xy, float2(12.9898, 78.233))) * 43758.5453) * 6.2831853;
            float occ = 0;
            for (int k = 0; k < 8; k++)
            {
                float a = ang + k * 2.3999632;          // 黄金角のらせん
                float h = (k + 0.5) / 8.0;
                float rr = sqrt(h);
                float3 dir = float3(cos(a) * rr, sin(a) * rr, -sqrt(1.0 - h));
                dir = dir - N * min(0.0, dot(dir, N)) * 2.0;   // 面の表側の半球へ折り返す
                float3 S = P + dir * r * (0.35 + 0.65 * h);
                float2 uv = (S.xy / S.z) * _Proj.zw * 0.5 + 0.5;
                float ds = EyeDepth(uv);
                float range = smoothstep(0.0, 1.0, r / max(1e-4, abs(P.z - ds)));
                occ += step(ds, S.z - r * 0.04) * range;
            }
            return 1.0 - occ / 8.0;
        }

        half4 fragDown(v2f_img i) : SV_Target
        {
            float2 d = _MainTex_TexelSize.xy;
            half3 c = (tex2D(_MainTex, i.uv + d * float2(-1, -1)).rgb + tex2D(_MainTex, i.uv + d * float2(1, -1)).rgb
                     + tex2D(_MainTex, i.uv + d * float2(-1, 1)).rgb + tex2D(_MainTex, i.uv + d * float2(1, 1)).rgb) * 0.25;
            return half4(min(c, 16.0), 1);
        }

        half4 fragBlur(v2f_img i) : SV_Target
        {
            float2 d = _Dir.xy * _MainTex_TexelSize.xy;
            half4 c = tex2D(_MainTex, i.uv) * 0.227
                    + (tex2D(_MainTex, i.uv + d * 1.385) + tex2D(_MainTex, i.uv - d * 1.385)) * 0.316
                    + (tex2D(_MainTex, i.uv + d * 3.231) + tex2D(_MainTex, i.uv - d * 3.231)) * 0.070;
            return c;
        }

        // ACES (RRT + ODT) の近似 (Stephen Hill)。HDRP・URP の Tonemapping (ACES) と同じ見え方
        float3 Aces(float3 c)
        {
            c = float3(dot(float3(0.59719, 0.35458, 0.04823), c), dot(float3(0.07600, 0.90834, 0.01566), c), dot(float3(0.02840, 0.13383, 0.83777), c));
            float3 a = c * (c + 0.0245786) - 0.000090537;
            float3 b = c * (0.983729 * c + 0.4329510) + 0.238081;
            c = a / b;
            c = float3(dot(float3(1.60475, -0.53108, -0.07367), c), dot(float3(-0.10208, 1.10813, -0.00605), c), dot(float3(-0.00327, -0.07276, 1.07602), c));
            return saturate(c);
        }
        float3 LinToLogC(float3 x) { return 0.244161 * log10(5.555556 * x + 0.047996) + 0.386036; }
        float3 LogCToLin(float3 x) { return (pow(10.0, (x - 0.386036) / 0.244161) - 0.047996) / 5.555556; }

        half4 fragFinal(v2f_img i) : SV_Target
        {
            float3 c = tex2D(_MainTex, i.uv).rgb;
            if (_Dof.z > 0.5)
            {
                float d = EyeDepth(i.uv);
                float coc = saturate((d - _Dof.x) / max(1e-3, _Dof.y - _Dof.x));
                c = lerp(c, tex2D(_Far, i.uv).rgb, smoothstep(0.0, 1.0, coc));
            }
            c *= lerp(1.0, tex2D(_AO, i.uv).r, _Params2.z);
            float3 bloom = (tex2D(_Bloom, i.uv).rgb + tex2D(_Bloom2, i.uv).rgb) * 0.5;
            c = lerp(c, bloom, _Params.x);
            // 周辺減光 (HDRP の Vignette と同じ式)
            float2 q = abs(i.uv - 0.5) * _Params2.x * 3.0;
            c *= pow(saturate(1.0 - dot(q, q)), _Params2.y * 5.0);
            c *= _Params.y;
            if (_Dof.w > 0.5)
            {
                // コントラスト (対数の中間の灰色を軸に)・彩度
                c = LogCToLin((LinToLogC(max(c, 0.0)) - 0.4135884) * _Params.z + 0.4135884);
                float l = dot(c, float3(0.2126729, 0.7151522, 0.0721750));
                c = max(0.0, l + (c - l) * _Params.w);
                c = Aces(c);
            }
            return half4(c, 1);
        }
        ENDCG
        Pass { CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment fragAO
            ENDCG }
        Pass { CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment fragDown
            ENDCG }
        Pass { CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment fragBlur
            ENDCG }
        Pass { CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment fragFinal
            ENDCG }
    }
}
