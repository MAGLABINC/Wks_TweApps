パケットエラー測定ツール・UART での使用方法
　　　　　　　　　　　　　　　　　　　　　　　　　　東京コスモス電機 2014/7/17


ファームウェアの書き込み等は readme_utf8.txt を参照ください。

(0) Master・Slaveを条件の良いごく近くに配置する。
(1) Masterの UART0, 115200 8N1 でターミナルを開く。
(2) Slave電源を入れる。
※ Master/Slave と試験可能なのは 1:1 のペアで、複数台のSlaveを同時に電源を
   投入する必要はありません。手順 (4) では、複数台のSlaveが存在するときに
   特定の１台を選択するようになっていますが、原則としてはSlave１台のみ電源
   を投入しいます。
(3) Masterをリセットまたは電源投入する。
(4) Slaveのアドレスが見つかったら 1-4 で選択する。
(5) 親子間で実験したい条件でテストを行う。
(6) テストを行った後、設定を変更せず、場所を移動し、
　’s’ キーにより試験を実施。これを繰り返す。

あとは各コマンドにより操作します。

'n' or 'N' --> RESCAN NODES
		ノードを再スキャンする
'0', '1-4' --> SHOW, CHOOSE NODES
		ノードリストの表示、選択を行う
'a' or 'A' --> SET APPACK
		AppAck モードのトグル
'c' or 'C' --> SET TX COUNT
		送信カウントの設定
'p' or 'P' --> SET PAYLOAD SIZE
		ペイロードサイズの設定
'>', '<'   --> CH UP, DOWN
		チャネルの設定
'r' or 'R' --> SET RETRY COUNT
		MAC 再送回数の設定
't' or 'T' --> SET TIMER ms
		送信周期の設定
'o' or 'O' --> SET Power
		送信出力の設定
's' or SPACE --> START PER TEST
		テストの開始
'b' or 'B'   --> STOP PER TEST
		テストの中断


[制限事項など]
・原則として LCD 付きの評価キットでの動作を前提とし、UART インタフェース
  では一部機能が欠落します(例：チャネルノイズ表示・受信強度表示)

　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　以上