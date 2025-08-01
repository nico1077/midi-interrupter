# midi-interrupter
基本的にテスラコイルを稼働させるために作成されたコードです。MIDI形式の音源を入力するとそれに対応したパルス波が出力されるコードです。ファンクションジェネレータ機能や単一のパルス波をパソコンなしで直接出力する機能もあります。  

This is basically a code created to operate a Tesla coil. When a MIDI format sound source is input, the corresponding pulse wave is output. It also has a function as a function to generate a function and to directly output a single pulse wave without a computer.  

## **使い方**
STM32CubeIDEで新しくプロジェクトを作成したあと、Core/Incフォルダの中にlcd_driver.hを挿入してください。Core/Srcフォルダの中にlcd_driver.cとmain.cをそれぞれ挿入するか上書きしてください。そしてプロジェクトファイル直下に.iocファイルを張り付けてください(プロジェクト名と.iocファイルをそろえることを忘れないでください。(例:interrupter_update/interrupter_update.ioc))。  
ファイルの導入が完了したら、ビルドしてエラーが出ないことを確認したらマイコンに書き込んで完了です。

NUCLEO-F446REというマイコンボードに書き込むことを想定しています。また、私が作成したコードはSTM32CubeIDEで書き込むことを前提としています。また、4行×20列のLCDモニタ(秋月電子通商にて販売されている型番:ACM2004D-FLW-FBW-IIC)に表示する機能を持っています。  
このコードでは4つの機能があり、MODE_PULSE、MODE_MIDI、MODE_MIDI_LOUDNESS、FG(ファンクションジェネレータ機能)です。  

### **0.FG(ファンクションジェネレータ機能)**  
これはほかの3つの機能のどう起動しているかに関わらず常に起動している機能です。テスラコイルをフィードバック制御ではなく直接共進周波数を入力するために使います。(もちろん未使用でも構いません)LCDモニタの一番下の行に常に周波数が表示され、周波数はPB_1(A3)ピンに接続した可変抵抗で粗調整し、PC_1(A4)ピンに接続した可変抵抗で微調整できます。なお、PA_8(D7)から出力されます。  

### **1.MODE_PULSE**  
これはPC_0(A5)ピンに何もしていないとき電源を入れると起動し、PA_1(A1)ピンに接続した可変抵抗でオンタイムのパルス幅の時間を、PA_4(A2)ピンに接続した可変抵抗で周波数を調整できます。パルス信号はPA_6(D12)ピンから出力されます。  

### **2.MODE_MIDI**  
これはPA_10(D2)ピンに6n137などのフォトカプラを介してMIDI信号を入力し、順番にPC_6,PA_5,PA_6,PB_6,PA_0,PA_7,PA_2,PB12ピンからそれぞれ単一のパルス波が出力されます。PA_1(A1)ピンに接続した可変抵抗を回すとduty比が調整できます。のPC_0(A5)ピンをGNDと接続したまま、PA_4(A2)ピンに接続した可変抵抗を最低値に振った状態で電源を入れるとMODE_PULSEのかわりにこのMODE_MIDIが起動します。出力された信号をORゲートやダイオードORで合成すればミックスして利用できます。(テスラコイルを稼働させるならパルス幅制限回路を介して信号を入力することをお勧めします。)  

### **3.MODE_MIDI_LOUDNESS**  
これはMODE_MIDIとほとんど同じで、PA_4(A2)ピンに接続した可変抵抗を最低値の代わりに最大値に振って電源を入れると起動します。MODE_MIDIではどの周波数の音も同じduty比で制御していましたが、高音のほうが音が通りやすく低音のほうが音が通りにくいため全く低音が聞こえなくなる可能性があるため、ラの音:MIDIノートの番号は69(440Hz)を基準とし、これより音が高ければduty比を小さくし、これより音が低ければduty比を大きくする関数が掛けられているだけです。つまり、f(69)=1を満たす関数f(x)をPA_1(A1)ピンに接続した可変抵抗から読み取った値と掛け算して調整しているだけです。自分好みにf(x)を変更しましょう！  

最後に、私が作成したコードはSTM32CubeIDEで書き込むことを前提としています。また、私はプログラマー初心者でGeminiAIを多用しました。完成したコードにGeminiAIでコードの中身を説明するコメントアウトを自動生成してもらっていますが、コードがおかしいところや無駄な処理があるかもしれませんが、その時は教えていただければ幸いです。  
それでは良いテスラコイルライフを！
