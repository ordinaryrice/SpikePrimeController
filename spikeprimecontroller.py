#SPIKE Controller
#author：HF@ordinaryrice
#使い方
#   SPIKEアプリ（バージョン2.x.x）で「新しいプロジェクト」→「Python」を選んで、
#   このコードを全てコピー＆ペーストします。
#   PoweredUPやControl+のハブ（電池ボックス）にLモーターまたはXLモーターを合計2個つなぎます。
#   SPIKEハブに圧力センサーとカラーセンサーをつないで左側を下にして横向きに持ち、
#   SPIKEハブのライトマトリクスに回転する矢印が表示されたら電池ボックスの電源ボタンを押し、接続します。
#   ライトマトリクスにメーターが表示されたら接続完了、操作可能です。
#   終了するときはSPIKEハブの左/右ボタンを押してください。
#
#   片方のモーターを走行用（エンジンモーター）、もう一つのモーターをハンドル操作用（サーボ）として使います。
#   42109 レゴテクニック トップギア・ラリーカー（アプリコントロール）のような車を操作することを想定していますが、
#   自分はトップギア・ラリーカーを持っていないので動作確認は未検証です。走る向きやハンドルが逆になる場合はうまく調整してください。
#   「エンジンモーター制御関数」や「サーボモーター制御関数」を一部書き換えれば対応できるはずです。
#
#※注意※
#   トレインモーターやバットモービルなどのモーターは使えません。SPIKEプライムのモーターはたぶん大丈夫（未検証）。
#   プログラムを終了しても切断されない仕様？のようなので簡単に電池を外せるようにしておくと良いでしょう。
#   このプログラムの改変は自由です。
#
#参考文献
#micropython(SPIKEプライムのBluetoothを制御するライブラリ)：
#   https://micropython-docs-ja.readthedocs.io/ja/latest/library/ubluetooth.html
#LEGO Wireless Protocol 3.0.00（PoweredUPやControl+を制御するコマンドが載っています）：
#   https://lego.github.io/lego-ble-wireless-protocol-docs/index.html

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer

import ubluetooth
from sys import exit
from micropython import const
from ustruct import unpack

#パラメータ設定ここから
timeout  = 20  # Bluetooth接続のタイムアウト時間（秒）
interval = 0.1 # メインループの長さ（秒）

#PoweredUPやControl+に接続したモーターのポートを指定する
servo  = 1 # サーボモーター（ポート：A=0, B=1, C=2, D=3）
engine = 3 # エンジンモーター

#SPIKEプライムに接続したセンサーのポートを指定する
color = 'A' # カラーセンサー（ポート：A~F）
force = 'E' # 圧力センサー(ポート：A~F)

posMax = 150 # サーボの最大回転角（片側あたり）

inv_color = 'white'
#パラメータ設定ここまで

_IRQ_SCAN_RESULT                    = const(5)  # デバイススキャン結果取得
_IRQ_PERIPHERAL_CONNECT             = const(7)  # 接続完了
_IRQ_PERIPHERAL_DISCONNECT          = const(8)  # 切断完了
_IRQ_GATTC_SERVICE_RESULT           = const(9)  # サービススキャン結果取得
_IRQ_GATTC_CHARACTERISTIC_RESULT    = const(11) # キャラクタリスティックスキャン結果取得

class TechnicHub:
    service_uuid = ubluetooth.UUID('00001623-1212-efde-1623-785feabcd123') # PowerdUPやControl+のサービスUUID
    char_uuid    = ubluetooth.UUID('00001624-1212-efde-1623-785feabcd123') # キャラクタイスティックUUID
    
    def __init__(self):
        self.phub = PrimeHub() # プライムハブ初期化
        self.color = ColorSensor(color) # カラーセンサー
        self.force = ForceSensor(force) # 圧力センサー
        self.connected = False # デバイスに接続できたか？
        self.running = False # デバイス初期化完了したか？

        self.ble = ubluetooth.BLE() # Bluetoothを初期化
        self.ble.irq(self.handler) # コールバック関数を指定
        self.ble.active(True) # Bluetooth アクティブ
        self.conn_handle  = 0 # conn_handle
        self.value_handle = 0 # value_handle
        
        self.ble.gap_advertise() # アドバタイズ開始
        wait_for_seconds(1)
        
        self.ble.gap_scan(timeout * 1000) # 周辺のデバイスを検索
        for i in range(timeout): # 接続できたor時間切れまで待つ
            self.phub.light_matrix.show_image("ARROW_N")
            wait_for_seconds(0.25)
            self.phub.light_matrix.show_image("ARROW_E")
            wait_for_seconds(0.25)
            self.phub.light_matrix.show_image("ARROW_S")
            wait_for_seconds(0.25)
            self.phub.light_matrix.show_image("ARROW_W")
            wait_for_seconds(0.25)
            if self.connected: break # 接続完了したらforループを抜ける
        
        if not self.connected: exit() # 時間切れならプログラム終了
        wait_for_seconds(1) # 少し待つ
        
        while not self.running:
            wait_for_seconds(1) # デバイス初期化が済むまで待つ
        
        wait_for_seconds(1) # 少し待つ
        self.main() # メインループ

    def handler(self, event, data):
        if event == _IRQ_SCAN_RESULT: # アドバタイズ中
            if b'\x90\x84\x2B' == data[1][0:3]: # MACアドレスの左半分がPowerdUPやControl+なら
                self.ble.gap_connect(data[0], data[1]) # 接続する
                self.connected = True # 接続完了をメイン処理に伝える
        
        elif event == _IRQ_PERIPHERAL_CONNECT: # 接続出来たら
            self.conn_handle = data[0] # conn_handleを取得
            self.ble.gattc_discover_services(self.conn_handle) # サービス検索
            wait_for_seconds(1.5) # 少し待つ
        
        elif event == _IRQ_GATTC_SERVICE_RESULT: # サービスの検索結果
            if data[3] == TechnicHub.service_uuid: # 目的のサービスUUIDなら
                self.ble.gattc_discover_characteristics(self.conn_handle, 0x1, 0xffff) # キャラクタリスティック検索
        
        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT: # キャラクタリスティックの検索結果
            if data[4] == TechnicHub.char_uuid: # 目的のキャラクタリスティックUUIDなら
                self.value_handle = data[2] # value_handleを取得
                self.send(bytes([0x81, servo, 0x10, 0x05, 0xf4, 0x01, 0x01])) # サーボモーターの加速をなめらかになるように設定
                wait_for_seconds(0.5) # 少し待つ
                self.running = True # デバイス初期化完了、メインループへ
        
        elif event == _IRQ_PERIPHERAL_DISCONNECT: # 切断出来たら
            self.running   = False # メインループを終了する
            self.connected = False # 切断完了を切断待機ループに伝える
            exit() # プログラム終了

    def send(self, data): #PowerdUPやControl+にコマンド送信する関数
        try:
            self.ble.gattc_write(self.conn_handle,self.value_handle,(len(data)+2).to_bytes(2, 'little') + data)
        except OSError: # コマンド送信に失敗したら
            pass # スルー

    def accel(self): # エンジンモーター制御関数
        acc = self.force.get_force_percentage() # 圧力センサー読み取り（単位：％）
        # ライトマトリクスにメーターを表示
        if acc < 25:
            self.phub.light_matrix.show_image('CLOCK10')
        elif acc < 32:
            self.phub.light_matrix.show_image('CLOCK11')
        elif acc < 39:
            self.phub.light_matrix.show_image('CLOCK12')
        elif acc < 46:
            self.phub.light_matrix.show_image('CLOCK1')
        elif acc < 53:
            self.phub.light_matrix.show_image('CLOCK2')
        elif acc < 60:
            self.phub.light_matrix.show_image('CLOCK3')
        elif acc < 68:
            self.phub.light_matrix.show_image('CLOCK4')
        elif acc < 76:
            self.phub.light_matrix.show_image('CLOCK5')
        elif acc < 84:
            self.phub.light_matrix.show_image('CLOCK6')
        elif acc < 92:
            self.phub.light_matrix.show_image('CLOCK7')
        else:
            self.phub.light_matrix.show_image('CLOCK8')
        if self.color.get_color() == inv_color: # カラーセンサーが白なら
            acc = (256 - acc) % 256 # エンジンモーターの回転方向を逆にする
        self.send(bytes([0x81, engine, 0x10, 0x51, 0x0, acc])) # エンジンモーターにコマンド送信

    def steering(self): # サーボモーター制御関数
        pitch = self.phub.motion_sensor.get_pitch_angle() # PRIMEハブのピッチ角（-90～+90）を取得
        pos = pitch * posMax // 90      # ピッチ角をサーボの回転角にマッピング
        if pos > posMax: pos = posMax   # 角度がオーバーしないようにする
        if pos < -posMax: pos = -posMax # 角度がオーバーしないようにする
        self.send(bytes([0x81, servo, 0x10, 0x0d]) + pos.to_bytes(4, "little") + bytes([100, 50, 127, 1, 0x01])) # サーボモーターにコマンド送信

    def main(self): # メインループ
        dt = interval * 0.5    # ループ1回あたり2回コマンドを送信するので
        
        while self.running:    # メインループ
            self.accel()         # エンジンモーターにコマンドを送る
            wait_for_seconds(dt) # 少し待つ
            self.steering()      # サーボモーターにコマンドを送る
            wait_for_seconds(dt) # 少し待つ
            if self.phub.left_button.is_pressed() or self.phub.right_button.is_pressed(): # 右・左のボタンが押されたとき
                self.running = False # ループを止める
        
        self.ble.gap_disconnect(self.conn_handle) # 接続解除
        while self.connected:    # 切断待ちループ
            wait_for_seconds(1)  # 切断完了まで待機
        
        exit() # プログラム終了

technichub = TechnicHub() # プログラムを実行
