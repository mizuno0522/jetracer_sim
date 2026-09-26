// 車載カメラの後処理。Unity のピンホール描画を実カメラの見た目に寄せる。
//   レンズ歪み (OpenCV plumb_bob k1,k2) → 周辺減光 → ぼかし/モーションブラー → 露出 → ノイズ → 車体の柱 (画面下)
// 歪み: 出力画素 (u,v) → 正規化 (xd,yd) = ((u-cx)/fx, (v-cy)/fy) → 不動点反復で歪みを解いて (x,y) →
//       ピンホールで描いた RT (正規化範囲 _Tan = x0,x1,y0,y1) の uv をサンプル。式は jetracer_common/cam_geom.py と同じ。
// 幾何の数値は course.json の camera (定義元は vehicle_profile.camera)、見た目の数値は realism。
Shader "Minicar/SensorPost"
{
    Properties
    {
        _MainTex ("Source", 2D) = "white" {}
        _Distort ("Use camera intrinsics (0/1)", Float) = 0
        _Intr ("fx fy cx cy [px]", Vector) = (64.66, 64.66, 112, 112)
        _Dist ("k1 k2 rmax^2 -", Vector) = (0, 0, 1000000, 0)
        _Tan ("render tan x0 x1 y0 y1", Vector) = (-1.732, 1.732, -1.732, 1.732)
        _OutSize ("output W H", Vector) = (224, 224, 0, 0)
        _Vignette ("Vignette strength", Float) = 0.15
        _BlurPx ("Blur radius [src px]", Float) = 1.0
        _MotionPx ("Horizontal motion blur [src px]", Float) = 0.0
        _Exposure ("Exposure gain", Float) = 1.0
        _Gamma ("Gamma", Float) = 1.0
        _Noise ("Noise sigma [0-1]", Float) = 0.012
        _Seed ("Noise seed", Float) = 0.0
        _PostDark ("Post darkness", Float) = 0.15
        _Post0 ("Post 0: x0 x1 y0 soft", Vector) = (0.24, 0.26, 0.93, 0.01)
        _Post1 ("Post 1: x0 x1 y0 soft", Vector) = (0.54, 0.56, 0.93, 0.01)
    }
    SubShader
    {
        Cull Off ZWrite Off ZTest Always
        Pass
        {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _MainTex;
            float4 _MainTex_TexelSize;
            float _Distort;
            float4 _Intr, _Dist, _Tan, _OutSize;
            float _Vignette, _BlurPx, _MotionPx, _Exposure, _Gamma, _Noise, _Seed, _PostDark;
            float4 _Post0, _Post1;

            float hash(float2 p)
            {
                float3 p3 = frac(float3(p.xyx) * 0.1031);
                p3 += dot(p3, p3.yzx + 33.33);
                return frac((p3.x + p3.y) * p3.z);
            }

            float post(float2 uv, float4 p)
            {
                // uv.y は上が 1。柱は画面下 (uv.y < 1 - y0)
                float inx = smoothstep(p.x - p.w, p.x + p.w, uv.x) * (1.0 - smoothstep(p.y - p.w, p.y + p.w, uv.x));
                float iny = 1.0 - smoothstep(1.0 - p.z - p.w, 1.0 - p.z + p.w, uv.y);
                return inx * iny;
            }

            fixed4 frag(v2f_img i) : SV_Target
            {
                // --- レンズ歪み: 出力画素 → 歪んだ正規化座標 → 反復で歪みを解く → ピンホール RT の uv ---
                float2 uv = i.uv;
                if (_Distort > 0.5)
                {
                    float u = i.uv.x * _OutSize.x;              // 左上原点・画素中心 +0.5 (uv は画素中心で評価される)
                    float v = (1.0 - i.uv.y) * _OutSize.y;
                    float xd = (u - _Intr.z) / _Intr.x, yd = (v - _Intr.w) / _Intr.y;
                    float x = xd, y = yd;
                    [unroll] for (int it = 0; it < 8; it++)
                    {
                        float rr = min(x * x + y * y, _Dist.z);
                        float g = 1.0 + _Dist.x * rr + _Dist.y * rr * rr;
                        x = xd / g; y = yd / g;
                    }
                    uv = float2((x - _Tan.x) / (_Tan.y - _Tan.x), 1.0 - (y - _Tan.z) / (_Tan.w - _Tan.z));
                }
                float2 c = (i.uv - 0.5) * 2.0;                  // 周辺減光は出力画像の座標で
                float r2 = dot(c, c);
                float inside = step(0.0, uv.x) * step(uv.x, 1.0) * step(0.0, uv.y) * step(uv.y, 1.0);

                // --- ぼかし (5 タップ) + 横方向のモーションブラー (3 タップ) ---
                float2 t = _MainTex_TexelSize.xy;
                float b = _BlurPx;
                float3 col = tex2D(_MainTex, uv).rgb * 0.4;
                col += tex2D(_MainTex, uv + float2( b, 0) * t).rgb * 0.15;
                col += tex2D(_MainTex, uv + float2(-b, 0) * t).rgb * 0.15;
                col += tex2D(_MainTex, uv + float2(0,  b) * t).rgb * 0.15;
                col += tex2D(_MainTex, uv + float2(0, -b) * t).rgb * 0.15;
                if (_MotionPx > 0.01)
                {
                    float3 m = tex2D(_MainTex, uv + float2(_MotionPx, 0) * t).rgb
                             + tex2D(_MainTex, uv - float2(_MotionPx, 0) * t).rgb
                             + tex2D(_MainTex, uv + float2(_MotionPx * 0.5, 0) * t).rgb
                             + tex2D(_MainTex, uv - float2(_MotionPx * 0.5, 0) * t).rgb;
                    col = (col + m) / 5.0;
                }

                // --- 周辺減光 (cos^4 風) と露出 ---
                float vig = 1.0 - _Vignette * saturate(r2);
                col = pow(saturate(col * vig * _Exposure), _Gamma);

                // --- センサノイズ ---
                float n = hash(i.uv * 977.0 + _Seed) - 0.5;
                col += n * _Noise * 2.0;

                // --- 画面外 (歪みで引き延ばした端) と車体の柱 ---
                col = lerp(float3(0.05, 0.05, 0.05), col, inside);
                float pm = max(post(i.uv, _Post0), post(i.uv, _Post1));
                col = lerp(col, float3(_PostDark, _PostDark, _PostDark), pm);
                return fixed4(saturate(col), 1.0);
            }
            ENDCG
        }
    }
}
