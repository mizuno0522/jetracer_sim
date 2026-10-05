// 画面表示のカメラに付ける後処理 (Shaders/ViewPost.shader)。Built-in の描画で HDRP 版と同じ仕様の絵を出す。
// 車載カメラ (センサ画像) と真上からの全景 (図として読むもの) には付けない。-quality low では付けない。
// 何をどの強さで掛けるかは RenderCompat.GetLook (3 版で共通)。付けるのは RenderCompat.Builtin.cs
using UnityEngine;

namespace Minicar
{
    public class ViewPost : MonoBehaviour
    {
        Material m_Mat;
        Look m_Look;
        Camera m_Cam;

        public static void Attach(Camera cam, Look look)
        {
            if (cam == null || !look.post || cam.GetComponent<ViewPost>() != null) return;
            var src = Resources.Load<Material>("Mat_ViewPost");
            if (src == null) { Debug.LogError("[ViewPost] Mat_ViewPost が無い (MinicarBuild.MakeMaterials)"); return; }
            cam.allowHDR = true;        // 1 より明るい所 (白線・塗装のハイライト・窓) を保ったままトーンカーブに渡す
            if (look.ao > 0f || look.dof) cam.depthTextureMode |= DepthTextureMode.Depth;
            var v = cam.gameObject.AddComponent<ViewPost>();
            v.m_Cam = cam;
            v.m_Look = look;
            v.m_Mat = new Material(src);
        }

        void Blur(RenderTexture a, int iterations, float scale)
        {
            var b = RenderTexture.GetTemporary(a.width, a.height, 0, a.format);
            for (int k = 0; k < iterations; k++)
            {
                m_Mat.SetVector("_Dir", new Vector4((1f + k) * scale, 0f, 0f, 0f));
                Graphics.Blit(a, b, m_Mat, 2);
                m_Mat.SetVector("_Dir", new Vector4(0f, (1f + k) * scale, 0f, 0f));
                Graphics.Blit(b, a, m_Mat, 2);
            }
            RenderTexture.ReleaseTemporary(b);
        }

        void OnRenderImage(RenderTexture src, RenderTexture dst)
        {
            if (m_Mat == null) { Graphics.Blit(src, dst); return; }
            var k = m_Look;
            var p = m_Cam.projectionMatrix;
            m_Mat.SetVector("_Proj", new Vector4(1f / p.m00, 1f / p.m11, p.m00, p.m11));
            m_Mat.SetVector("_Params", new Vector4(k.bloom, k.exposure, 1f + k.contrast * 0.01f, 1f + k.saturation * 0.01f));
            m_Mat.SetVector("_Params2", new Vector4(k.vignette, k.vignetteSmooth, k.ao, k.aoRadius));
            m_Mat.SetVector("_Dof", new Vector4(k.dofStart, k.dofEnd, k.dof ? 1f : 0f, 1f));
            int w = src.width, h = src.height;

            // にじみ: 1/4 と 1/16 の 2 段のぼかし
            var b1 = RenderTexture.GetTemporary(Mathf.Max(8, w / 4), Mathf.Max(8, h / 4), 0, src.format);
            var b2 = RenderTexture.GetTemporary(Mathf.Max(8, w / 16), Mathf.Max(8, h / 16), 0, src.format);
            b1.filterMode = b2.filterMode = FilterMode.Bilinear;
            RenderTexture far = null, ao = null;
            if (k.bloom > 0f || k.dof)
            {
                var half = RenderTexture.GetTemporary(Mathf.Max(8, w / 2), Mathf.Max(8, h / 2), 0, src.format);
                Graphics.Blit(src, half, m_Mat, 1);
                if (k.dof)
                {
                    far = RenderTexture.GetTemporary(half.width, half.height, 0, src.format);
                    Graphics.Blit(half, far);
                    Blur(far, 2, k.dofBlur / 3.5f * h / 1080f);
                    m_Mat.SetTexture("_Far", far);
                }
                Graphics.Blit(half, b1, m_Mat, 1);
                RenderTexture.ReleaseTemporary(half);
                Blur(b1, 2, 1f);
                Graphics.Blit(b1, b2, m_Mat, 1);
                Blur(b2, 2, 1f);
            }
            m_Mat.SetTexture("_Bloom", b1);
            m_Mat.SetTexture("_Bloom2", b2);
            if (k.ao > 0f)
            {
                ao = RenderTexture.GetTemporary(Mathf.Max(8, w / 2), Mathf.Max(8, h / 2), 0, RenderTextureFormat.R8, RenderTextureReadWrite.Linear);
                Graphics.Blit(src, ao, m_Mat, 0);
                Blur(ao, 1, 1f);
                m_Mat.SetTexture("_AO", ao);
            }
            else m_Mat.SetTexture("_AO", Texture2D.whiteTexture);
            Graphics.Blit(src, dst, m_Mat, 3);
            RenderTexture.ReleaseTemporary(b1);
            RenderTexture.ReleaseTemporary(b2);
            if (far != null) RenderTexture.ReleaseTemporary(far);
            if (ao != null) RenderTexture.ReleaseTemporary(ao);
        }
    }
}
