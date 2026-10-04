// 画面表示のカメラに付ける後処理 (Shaders/ViewPost.shader)。車載カメラ (センサ画像) には付けない。
// -quality low (学習) とミニカーの会場では付けない。
using UnityEngine;

namespace Minicar
{
    public class ViewPost : MonoBehaviour
    {
        Material m_Mat;

        /// 実車スケールのコースの表示用カメラに付ける (low では何もしない)
        public static void Attach(Camera cam)
        {
            if (cam == null || RenderQuality.Current == QualityTier.Low || cam.GetComponent<ViewPost>() != null) return;
            var src = Resources.Load<Material>("Mat_ViewPost");
            if (src == null) return;
            cam.allowHDR = true;        // 明るい所 (白線・塗装のハイライト・雪) を 1 より上まで持たせて、にじませる
            var v = cam.gameObject.AddComponent<ViewPost>();
            v.m_Mat = new Material(src);
            v.m_Mat.SetVector("_Params", new Vector4(1.05f, 0.28f, 0.38f, 0.96f));
            v.m_Mat.SetVector("_Params2", new Vector4(0.22f, 0.018f, 0f, 0f));
        }

        void OnRenderImage(RenderTexture src, RenderTexture dst)
        {
            if (m_Mat == null) { Graphics.Blit(src, dst); return; }
            int w = Mathf.Max(8, src.width / 4), h = Mathf.Max(8, src.height / 4);
            var a = RenderTexture.GetTemporary(w, h, 0, src.format);
            var b = RenderTexture.GetTemporary(w, h, 0, src.format);
            Graphics.Blit(src, a, m_Mat, 0);
            for (int k = 0; k < 2; k++)
            {
                m_Mat.SetVector("_Dir", new Vector4(1f + k, 0f, 0f, 0f));
                Graphics.Blit(a, b, m_Mat, 1);
                m_Mat.SetVector("_Dir", new Vector4(0f, 1f + k, 0f, 0f));
                Graphics.Blit(b, a, m_Mat, 1);
            }
            m_Mat.SetTexture("_Bloom", a);
            Graphics.Blit(src, dst, m_Mat, 2);
            RenderTexture.ReleaseTemporary(a);
            RenderTexture.ReleaseTemporary(b);
        }
    }
}
