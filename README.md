# ESPnowRC

ESP32によるラジコン送受信機＋FETスピコン

このコードで使っているESP32ボードは秋月のこちらです。
https://akizukidenshi.com/catalog/g/gK-16108/

このボードとサーボへの5V電圧供給は適切なDC-DCコンバータを使用してください。必要ならばパスコンを入れてください。

このコードで対応するモーターは小型のコアレスモータを想定しています。が，あくまでPWMコマンドですのでブラシDCモータとそれに対応するFET（と必要に応じて追加のフリーホイールダイオードとキャパシタ）であれば対応可能です。
http://flightlogbook.cocolog-nifty.com/logbook/2021/12/post-ba25a0.html

上記のKvを得るためも含む実験からは最大電圧2.5Vが適当として，電池電圧に関係なくパワーのコマンドがフルの時に2.5VとなるようにPWMデューティー比を調整するプログラムとしています。


    Copyright (C) 2023− Koichi Takasaki
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
