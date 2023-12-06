# ESPnowRC
このコードで使っているESP32ボードは秋月のこちらです。
https://akizukidenshi.com/catalog/g/gK-16108/

このボードとサーボへの5V電圧供給は適切なDC-DCコンバータを使用してください。必要ならばパスコンを入れてください。

このコードで対応するモーターは小型のコアレスモータを想定しています。が，あくまでPWMコマンドですのでブラシDCモータとそれに対応するFET（と必要に応じて追加のフリーホイールダイオードとキャパシタ）であれば対応可能です。
http://flightlogbook.cocolog-nifty.com/logbook/2021/12/post-ba25a0.html

上記のKvを得るためも含む実験からは最大電圧2.5Vが適当として，電池電圧に関係なくパワーのスティックがフルの時に2.5VとなるようにPWMデューティー比を調整するプログラムとしています。
