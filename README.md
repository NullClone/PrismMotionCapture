# Prism Motion Capture

Unityでウェブカメラを使用して高精度なモーショントラッキングを実行します。

## 使用方法

このリポジトリをクローン、またはダウンロードします。<br>
Unity 2022.3以上のバージョンでプロジェクトを開きます。<br>

次に、必要なパッケージをインポートする必要があります。<br>

[<b>FinalIK</b>](https://assetstore.unity.com/packages/tools/animation/final-ik-14290)

バージョン<b>2.4</b>以上をサポートしています。（最新のバージョンを使用することをおすすめします）<br>

1. Package ManagerなどからFinalIKをインポートします。<br>
2. インポートした後、<b>Plugins/RootMotion</b>フォルダ内にある<b>Import Assembly Definitions</b>パッケージをダブルクリックしてインポートします。

> デモフォルダなどは必須ではないため、インポート時に除外することもできます。

[<b>MediaPipeUnityPlugin</b>](https://github.com/homuler/MediaPipeUnityPlugin)

バージョン<b>0.16.1</b>以上をサポートしています。（最新のバージョンを使用することをおすすめします）<br>

1. リソースページから<b>MediaPipeUnity.[Version].unitypackage</b>をダウンロードします。<br>
2. ダウンロードしたパッケージをUnityにインポートします。

> サンプルが必要ない場合はインポート時に<b>Packages/com.github.homuler.mediapipe</b>のみにチェックをつけてインポートしてください。

<b>※以下のパッケージはすでに追加されています。</b>

[<b>UniVRM</b>](https://github.com/vrm-c/UniVRM)

こちらのパッケージはデモで使用されているアバターに必要なものですので、このリポジトリに直接関係があるわけではありません。
