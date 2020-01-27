# ex_cvgl

## 顔ジェスチャーによるPC操作補助機能

## 概要

アクティブウィンドウに対して、顔の動きによる簡単な操作を実現した。  
手が埋まっているときにもpdfを読む程度の操作ができる。  
料理中手が汚れていてもレシピを快適に閲覧できる。  

## Usage

・コンパイル、実行  
$cd cvgl/FaceTracker  
$make  
$cd bin  
$./face_tracker  
  
・マニュアル  
上を向く→上にスクロール  
下を向く→下にスクロール  
右を向く→NextWindow （Cmd + RightArrowに対応）  
左を向く→PreviousWindow （Cmd + LeftArrowに対応）  
目を閉じる→終了  

## reference  
実装にはFaceTrackerを用いた．
