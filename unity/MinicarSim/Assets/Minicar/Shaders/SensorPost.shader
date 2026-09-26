// 車載カメラの後処理。Unity のピンホール描画を実カメラの見た目に寄せる。
//   樽型歪み (Brown k1,k2) → 周辺減光 → ぼかし/モーションブラー → 露出 → ノイズ → 車体の柱 (画面下)
// 数値は course.json の realism (定義元は vehicle_profile.camera.realism)。
Shader "Minicar/SensorPost"
{
    Properties
    {
        _MainTex ("Source", 2D) = "white" {}
        _K1 ("Distortion k1", Float) = -0.08
        _K2 ("Distortion k2", Float) = 0.0
        _Zoom ("Zoom after distortion", Float) = 1.0
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
            float _K1, _K2, _Zoom, _Vignette, _BlurPx, _MotionPx, _Exposure, _Gamma, _Noise, _Seed, _PostDark;
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
                // --- 樽型歪み: 出力画素 → 歪んだ入力座標 (Brown、正規化半径) ---
                float2 c = (i.uv - 0.5) * 2.0 / _Zoom;
                float r2 = dot(c, c);
                float2 d = c * (1.0 + _K1 * r2 + _K2 * r2 * r2);
                float2 uv = d * 0.5 + 0.5;
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
                float vig = 1.0 - _Vignette * saturate(r2 * _Zoom * _Zoom);
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
