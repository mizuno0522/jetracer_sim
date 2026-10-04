// 画面表示 (追従視点など) の後処理。車載カメラ (配信するセンサ画像) には掛けない。
//   明るい所のにじみ (ブルーム) → コントラスト (S 字) → 彩度 → 陰は少し青く・光は少し暖かく → 周辺減光
// Built-in の描画で「光と空気感」を足すためのもの (-quality medium 以上・実車スケールのコースだけ)。HDRP 版では使わない
Shader "Minicar/ViewPost"
{
    Properties
    {
        _MainTex ("Source", 2D) = "white" {}
        _Bloom ("Bloom", 2D) = "black" {}
    }
    SubShader
    {
        Cull Off ZWrite Off ZTest Always
        CGINCLUDE
        #include "UnityCG.cginc"
        sampler2D _MainTex, _Bloom;
        float4 _MainTex_TexelSize;
        float4 _Dir;        // ぼかしの向き (テクセル)
        float4 _Params;     // x = ブルームのしきい値, y = ブルームの強さ, z = コントラスト, w = 彩度
        float4 _Params2;    // x = 周辺減光, y = 色の振り分け

        half4 fragPre(v2f_img i) : SV_Target
        {
            float2 d = _MainTex_TexelSize.xy;
            half3 c = (tex2D(_MainTex, i.uv + d * float2(-1, -1)).rgb + tex2D(_MainTex, i.uv + d * float2(1, -1)).rgb
                     + tex2D(_MainTex, i.uv + d * float2(-1, 1)).rgb + tex2D(_MainTex, i.uv + d * float2(1, 1)).rgb) * 0.25;
            half l = max(c.r, max(c.g, c.b));
            c *= saturate((l - _Params.x) / max(1e-3, 1.0 - _Params.x));
            return half4(min(c, 4.0), 1);
        }

        half4 fragBlur(v2f_img i) : SV_Target
        {
            float2 d = _Dir.xy * _MainTex_TexelSize.xy;
            half3 c = tex2D(_MainTex, i.uv).rgb * 0.227
                    + (tex2D(_MainTex, i.uv + d * 1.385).rgb + tex2D(_MainTex, i.uv - d * 1.385).rgb) * 0.316
                    + (tex2D(_MainTex, i.uv + d * 3.231).rgb + tex2D(_MainTex, i.uv - d * 3.231).rgb) * 0.070;
            return half4(c, 1);
        }

        half4 fragFinal(v2f_img i) : SV_Target
        {
            half3 c = tex2D(_MainTex, i.uv).rgb + tex2D(_Bloom, i.uv).rgb * _Params.y;
            c = saturate(c);
            c = lerp(c, c * c * (3.0 - 2.0 * c), _Params.z);
            half l = dot(c, half3(0.299, 0.587, 0.114));
            c = lerp(l.xxx, c, _Params.w);
            c += (l - 0.5) * half3(1, 0.2, -1) * _Params2.y;
            float2 q = (i.uv - 0.5) * 1.414;
            c *= 1.0 - _Params2.x * pow(dot(q, q), 1.1);
            return half4(saturate(c), 1);
        }
        ENDCG
        Pass { CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment fragPre
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
