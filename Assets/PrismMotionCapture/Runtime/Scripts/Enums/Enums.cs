namespace PMC
{
    public enum SourceType
    {
        Texture,
        Video,
        Webcam,
    }

    public enum RenderMode
    {
        None,
        RenderTexture,
        Renderer,
        RawImage,
    }

    public enum ImageReadMode
    {
        CPU,
        GPU,
    }

    public enum IKType
    {
        VRIK,
        FBBIK,
    }
}