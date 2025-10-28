using Google.Protobuf;
using Mediapipe;
using Mediapipe.Unity;

namespace PMC
{
    public static class MediaPipeExtensions
    {
        public static T Get<T>(this OutputStream<T>.OutputEventArgs eventArgs) where T : IMessage<T>, new()
        {
            if (eventArgs.packet == null) return default;

            var parser = new MessageParser<T>(() => new T());

            var result = eventArgs.packet.Get(parser);

            if (result == null) return default;

            return result;
        }
    }
}