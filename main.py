import mpu6050
import time
import komplementaerfilter
import bno055
import kalmanfilter
import math
import csv
import os
from flask import Flask, render_template         #für html übergabe
from flask_socketio import SocketIO              #für html übergabe




app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')                    #Initalisierung des WEbSockets , async_mode erlaubt parallele arbeit
log_data = []


@app.route('/')
def index():
    return render_template('index.html')

def sensor_loop():
    print('sensor loop gestartet')



    #kreiren der objekten, aufrufen des konstruktors
    Messwerte = mpu6050.MPU6050(adress=0x68)
    MessBno = bno055.BNO055(adress = 0x28)
    kompfilter_werte = komplementaerfilter.KOMPFILTER()

    ########################################################
    #MPU6050

    #berechner der start offset
    offset_x = Messwerte.offset(Messwerte.get_gyrox)
    offset_y = Messwerte.offset(Messwerte.get_gyroy)
    offset_z = Messwerte.offset(Messwerte.get_gyroz)
    offset_x_acc = Messwerte.offset(Messwerte.get_accx)
    offset_y_acc = Messwerte.offset(Messwerte.get_accy)
    offset_z_acc = Messwerte.offset(Messwerte.get_accz)

    #########################################################
    #BNO0055
    #gyrodaten lesen

    roll_adresse_h, roll_adresse_l =  0x1F, 0x1E
    pitch_adresse_h, pitch_adresse_l = 0x1D, 0x1C
    yaw_adresse_h, yaw_adresse_l = 0x1B, 0x1A

    #quaternionendaten änderung
    quatw_h, quatw_l = 0x21, 0x20
    quatx_h, quatx_l = 0x23, 0x22
    quaty_h, quaty_l = 0x25, 0x24
    quatz_h, quatz_l = 0x27, 0x26


    offs_yaw = MessBno.geteuler(yaw_adresse_h, yaw_adresse_l)
    #print(f"Offset-Yaw: {offs_yaw:.2f} Grad")
    ########################################################
    #Auslesen der gyro werte mit abzug des inital berechneten offsets
    Gyroskop_X = Messwerte.get_gyrox()
    Gyroskop_Y = Messwerte.get_gyroy()
    Gyroskop_Z = Messwerte.get_gyroz()

    #auslesen der acc werte mit abzug der iital berechneten offsets
    Acceleration_X = Messwerte.get_accx()
    Acceleration_Y = Messwerte.get_accy()
    Acceleration_Z = Messwerte.get_accz()

    kalman = kalmanfilter.KALMANFILTER(math.radians(Gyroskop_X),math.radians(Gyroskop_Y),math.radians(Gyroskop_Z),Acceleration_X,Acceleration_Y,Acceleration_Z)

    #########################################################
    zeitv = time.perf_counter()
    zeit_csv = zeitv
    
    yaw_start_offset = 0
    warmup_count = 0
    warmup_limit = 100
    first_run = False

    
    while True:
        
        #kreiren des Zeitschrittes dt
        zeit = time.perf_counter()
        dt = zeit - zeitv
        zeitv = zeit
        #absolute zeit fuer csv export
        absolute_zeit = zeit - zeit_csv

        #Auslesen der gyro werte mit abzug des inital berechneten offsets
        Gyroskop_X = Messwerte.get_gyrox()-offset_x
        Gyroskop_Y = Messwerte.get_gyroy()-offset_y
        Gyroskop_Z = Messwerte.get_gyroz()-offset_z

        #auslesen der acc werte mit abzug der iital berechneten offsets
        Acceleration_X = Messwerte.get_accx()-offset_x_acc
        Acceleration_Y = Messwerte.get_accy()-offset_y_acc
        Acceleration_Z = Messwerte.get_accz()-(offset_z_acc-1)
        
        #auslesen der 3 Eulerwinkel der IMU mittel komplementärfilter
        Roll, Pitch, Yaw = kompfilter_werte.kompfilter(Gyroskop_X,Gyroskop_Y,-Gyroskop_Z,Acceleration_X,Acceleration_Y,Acceleration_Z,dt)

        #auslesen der quaterionen mittels Kalmanfiler
        kalman.prediction(math.radians(Gyroskop_X),math.radians(Gyroskop_Y),math.radians(Gyroskop_Z), dt)
        X_corr, P_corr = kalman.correction(Acceleration_X,Acceleration_Y,Acceleration_Z)

        

        q0 = X_corr[0,0]
        q1 = X_corr[1,0]
        q2 = X_corr[2,0]
        q3 = X_corr[3,0]

        euler_kalman = kalman.geteuler()

        roll_kalman = euler_kalman[0,0]
        pitch_kalman = euler_kalman[1,0]
        yaw_kalman_roh = euler_kalman[2,0]

        if not first_run:
            warmup_count += 1
            yaw_kalman = 0
            

            if warmup_count >= warmup_limit:
                yaw_start_offset = yaw_kalman_roh # Der allererste Yaw-Wert des Kalman
                print(f"System bereit! Kalman-Offset: {yaw_start_offset:.2f}")
                first_run = True
        else:
            yaw_kalman = yaw_kalman_roh - yaw_start_offset
            yaw_kalman = (yaw_kalman + 180) % 360 - 180


        #######################################
        #daten des BNO055
        Roll_B = MessBno.geteuler(roll_adresse_h, roll_adresse_l)
        Pitch_B = MessBno.geteuler(pitch_adresse_h, pitch_adresse_l)
        Yaw_B = MessBno.geteuler(yaw_adresse_h, yaw_adresse_l) - offs_yaw
        Yaw_B = (Yaw_B + 180) % 360 - 180

        #quaternionnen
        q0_b = MessBno.getquaternion(quatw_h, quatw_l)
        q1_b = MessBno.getquaternion(quatx_h, quatx_l)
        q2_b = MessBno.getquaternion(quaty_h, quaty_l)
        q3_b = MessBno.getquaternion(quatz_h, quatz_l)




        ########################################
        #Display MPU6050

        #print(f"Gyro X: {Gyroskop_X:5.2f} °/s | Gyro Y: {Gyroskop_Y:5.2f} °/s | Gyro Z: {Gyroskop_Z:5.2f} °/s  | Acc X: {Acceleration_X:5.2f} g/s | Acc Y: {Acceleration_Y:5.2f} g/s | Acc Z: {Acceleration_Z:5.2f} g/s  |Pitch: {Pitch:5.2f} ° | Roll: {Roll:5.2f} °|Yaw: {Yaw:5.2f} °| dt: {dt:5.2f} s", end="\r")
        #print(f"Pitch: {Pitch:5.2f} ° | Roll: {Roll:5.2f} ° | dt: {dt:5.2f} s  ", end="\r")
        #print(f"|Pitch: {Pitch:5.2f} ° |Roll: {Roll:5.2f} ° |Yaw: {Yaw:5.2f} ° | dt: {dt:5.2f} s", end="\r")

        #Display Kalmanfilter
        #print(f"Lage (q0,q1,q2,q3): {q0:.3f} | {q1:.3f} | {q2:.3f} | {q3:.3f}")
        #print(f"|Lage q0: {q0:.3f}  |Lage q1: {q1:.3f}  |Lage q2: {q2:.3f}  | Lage q3: {q3:.3f} |Lage Bq0: {q0_b:.3f}  |Lage Bq1: {q1_b:.3f}  |Lage Bq2: {q2_b:.3f}  | Lage Bq3: {q3_b:.3f}", end="\r")

        #Display BNO055
        #print(f"|Pitch: {Pitch_B:5.2f} ° |Roll: {Roll_B:5.2f} ° |Yaw: {Yaw_B:5.2f} ° ", end="\r")

        ###################################################33
        #uebergae der werte in html

        socketio.emit('sensor_data_kalman', {
            'roll_kalman' : roll_kalman,
            'pitch_kalman' : pitch_kalman,
            'yaw_kalman' : yaw_kalman
        })

        socketio.emit('sensor_data_komp', {
            'Roll' : Roll,
            'Pitch' : Pitch,
            'Yaw' : Yaw
        })

        socketio.emit('sensor_data_bno055', {
            'Roll_B' : Roll_B,
            'Pitch_B' : Pitch_B,
            'Yaw_B' : Yaw_B
        })


        ##################################################
        #csv export, erweitern des array mit daten
        log_data.append([
        absolute_zeit, 
        Roll, Pitch, Yaw, 
        roll_kalman, pitch_kalman, yaw_kalman, 
        Roll_B, Pitch_B, Yaw_B
    ])

        ##################################################
        #verlgeicht vorgegebene periodendauer, mit (jetztige zeit mit zeit von beginn) -> verbleibende schalfzeit
        sleep_time = Messwerte.period - (time.perf_counter() - zeit)

        if sleep_time > 0:
            socketio.emit('check', {

                'check' : 'Pi rechnet genug schnell',
                'uebrige_zeit' : abs(sleep_time)
            })
            socketio.sleep(sleep_time)
        else:
            socketio.emit('warnung', {
                'warnung' : 'warnung rechenleistung am limit'
            })
            socketio.sleep(0)

            


if __name__ == '__main__':
    socketio.start_background_task(target = sensor_loop)        
    print('Start des Weswerer auf http://192.168.1.178:5000') 

    try:
        socketio.run(app, host='0.0.0.0', port=5000)


    except KeyboardInterrupt:
        print('\n Messung sauber beendet. ')

    finally:
        ##################
        #csv schreiben

        dateipfad = os.path.join("/home/pi/BA_IMU/data/", "messung_statisch_tuned_Qx100.csv")
        with open(dateipfad, mode='w', newline='') as file:
            writer = csv.writer(file)
            # header schreiben
            writer.writerow(["Zeit", "Roll_CF", "Pitch_CF", "Yaw_CF", "Roll_EKF", "Pitch_EKF", "Yaw_EKF", "Roll_BNO", "Pitch_BNO", "Yaw_BNO"])
            # daten schreiben
            writer.writerows(log_data)
        
        print("csv erfolgreich")

