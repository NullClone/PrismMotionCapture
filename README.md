# Prism Motion Capture

Unityで高精度なモーションキャプチャーを実現することができます。<br>
ウェブカメラや動画ファイルから、リアルタイムで手軽にフルトラッキングが可能です。<br>

## 使用方法

### 要件

本ライブラリには以下の環境が必要です。
- Unity 2022.3 以降
- FinalIK (有料アセット)
- MediaPipeUnityPlugin

### インストール

先に必要なパッケージをインストールすることをおすすめします。<br>

1. パッケージマネージャーを開きます `Window > Package Manager`

2. 左上の`+`ボタンから`Add package from git URL...`を選択します。

<p align="center">
  <img width="50%" src="https://github.com/user-attachments/assets/ed1fc738-0412-40e8-aa84-b32b643c31cb">
</p>

3. 以下のURLを入力します。
   ```bash
   https://github.com/NullClone/PrismMotionCapture.git?path=/Assets/PrismMotionCapture/
   ```

### 必要なパッケージ

[<b>FinalIK</b>](https://assetstore.unity.com/packages/tools/animation/final-ik-14290)

バージョン<b>2.4</b>以上をサポートしています。<br>

1. Package ManagerなどからFinalIKをインポートします。<br>
2. インポートした後、<b>Plugins/RootMotion</b>フォルダ内にある<b>Import Assembly Definitions</b>パッケージをダブルクリックでインポートします。<br>

[<b>MediaPipeUnityPlugin</b>](https://github.com/homuler/MediaPipeUnityPlugin)

バージョン<b>0.16.3</b>以上をサポートしています。（常に最新のバージョンを使用することをおすすめします）<br>

GitHubの[リリースページ](https://github.com/homuler/MediaPipeUnityPlugin/releases)から、最新の.unitypackageファイルをダウンロードし、Unityにインポートします。<br>

## 使い方

> 追加予定...<br>