class sers:
#
  def __init__(self):
    #read serials#    from read_m5b_class import m5logger
    import serial,time
    self.ser0=serial.Serial("/dev/ttyUSB0",115200) 
    self.ser1=serial.Serial("/dev/ttyUSB1",115200) 
    self.ser2=serial.Serial("/dev/ttyACM0",115200)
#   
  def read(self):
#    global data1,data2,data3,data4
    data1=data2=data3=[]
    while True:
      line01 = self.ser0.readline()
      line02 = self.ser1.readline()
      line03 = self.ser2.readline()
      try:
        line01s=line01.strip().decode('utf-8')
        line02s=line02.strip().decode('utf-8')
        line03s=line03.strip().decode('utf-8')
      except UnicodeDecodeError:
        continue
      data1s=line01s.split(",")
      data2s=line02s.split(",")
      data3s=line03s.split(",")
      if data1s[0]=="03": data1=data1s[2:12]
      if data1s[0]=="01": data2=data1s[2:12]
      data3=data1s[1:9]
      if data2s[0]=="03": data1=data2s[2:12]
      if data2s[0]=="01": data2=data2s[2:12]
      data3=data2s[1:9]
      if data3s[0]=="03": data1=data3s[2:12]
      if data3s[0]=="01": data2=data3s[2:12]
      data3=data3s[1:9]
#     
      return data1+data2+data3
  def close(self):
   self.ser0.close()
   self.ser1.close()
   self.ser2.close()

class thread_ssr:
  def __init__(self):   # initial action
    return
  def thread(self,it,q): # class body
    a=q.get()   # get Tc temp
#
    if time.time()-a[10]>=1800.: # freezer on/off
      a[10]=time.time()
      ssr18=0
    elif 0<time.time()-a[10]<=1500.:
      if a[0]<-20.0:   # freezer off -20と極端にしてみた {250865}
        ssr18=0
      else:
        ssr18=1 # 1->on, 0->off
    else:
      ssr18=0     
    # トータル 80000秒周期（1day=86400）での温度閾値切り替えロジックにした {250826}
    # 過冷却になっている可能性が高いので、-0.5 （薄い氷にしたかった）から、-2.0にしてみる {250628}
    time_elapsed = time.time() - a[11]
    if time_elapsed >= 80000.:
      a[11] = time.time()
      temp = -2.0  # 周期リセット時は-0.5に設定
    elif time_elapsed <= 70000.:
      temp = -2.0
    else:
      temp = 0.5
    av=sum(a[1:10])/9. # heater
    if av>temp:  # threshold of ssr switching  # 0.5 -> 0.0 24/JUl/2025 # 0.0->0.5 20250725
      ssr1=0   # ssr off
    else:
      ssr1=1   # ssr on
    print([ssr1,ssr18,a,f"time_elapsed:{time_elapsed:.1f}",f"temp:{temp}",f"av:{av:.2f}"])
    q.put([ssr1,ssr18,a[10],a[11]])   # set ssr value to queu
    return

ser=sers()
import time
from datetime import date
import matplotlib.pyplot as plt
import numpy as np
import threading
import queue  # library for queu operation
import RPi.GPIO as GPIO
#
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
#
GPIO.setup(11, GPIO.OUT)  # side heater: : ssr11
GPIO.setup(12, GPIO.OUT)  # core heater: ssr12
GPIO.setup(13, GPIO.OUT)  # heater: ssr13
GPIO.setup(15, GPIO.OUT)  # pump: ssr15
GPIO.setup(16, GPIO.OUT)  # side heater: ssr16
GPIO.setup(18, GPIO.OUT)  # freezer: ssr18
GPIO.setup(19, GPIO.OUT)  # bottom heater: ssr19
#
GPIO.output(15, 1)  # pump always runs
#
it=0   # thread counter
q =queue.Queue()  # queue which stores a result of a thread
ssr=[0,0,0]  # ssr[0]:ヒーター群, ssr[1]:冷凍庫, ssr[2]:COREヒーター(12番)
core_timer_start = time.time()
#
current_time = time.strftime("_H%H_M%M_S%S", time.localtime())
fn = "TAD_" + str(date.today()) + current_time + ".csv"
f=open(fn, 'w', encoding="utf-8")
start=time.time()
core_timer_start=start
c=[[0]*28]*10
x=range(0,10)
it=0
row=""
while 1:
 try:
  a=ser.read()
  try: 
    b=[float(val) for val in a]
  except:
    continue
  if len(a)==28:
   xlist=[]
   now = time.time()
   st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
   ss = str(time.time() - int(time.time()))
   xlist.append(1)
   xlist.append(2)
   row=st + ss[1:5] + "," + str(round(time.time()-start, 2)) + ","
   xlist=xlist+a+[0,0,0]
   if len(xlist)!=33:
     continue
   for i in range(0,27):
      row=row+a[i]+","
   row=row+a[27]
#
  if it==0:  #when thread has not started
    it=it+1
    thread1=thread_ssr() #provide a thread
    q.put(b[0:10]+[time.time()]+[time.time()])
    print(b[0:10]+[time.time()]+[time.time()])
    th = threading.Thread(target=thread1.thread,args=(it,q),daemon=True)
    th.start()
  if th.is_alive()==False:  #when no thread
     it=it+1
     ssr=q.get()
     ssrtime1=ssr[2]
     ssrtime2=ssr[3]
     thread1=thread_ssr() #provide a thread
     q.put(b[0:10]+[ssrtime1,ssrtime2])
     print(b[0:10]+[ssrtime1,ssrtime2])
     th = threading.Thread(target=thread1.thread,args=(it,q),daemon=True)
     th.start()
    # --- COREヒーター(12番)の独立制御 ---
  core_period = 100.0  # 秒
  core_on_time = 20.0  # ON時間
  now = time.time()
  elapsed = (now - core_timer_start) % core_period
  if elapsed < core_on_time:
      ssr[2] = 1  # ON
  else:
      ssr[2] = 0  # OFF
  GPIO.output(12, ssr[2])  # COREヒーター独立制御

    # set heaters ssr / ssr[0]==1: on;  ssr[0]==0: off (COREは除く)
  GPIO.output(11,ssr[0])  # side heater: : ssr11
    #GPIO.output(12,ssr[0])  # core heater: ssr12 ← 独立制御に変更
  GPIO.output(13,ssr[0])  # heater: ssr13
  GPIO.output(16,ssr[0])  # side heater: ssr16
  GPIO.output(19,ssr[0])  # bottom heater: ssr19
    # set freezer on / ssr[1]==1: on;  ssr[1]==0: off
  GPIO.output(18,ssr[1])
    #
  row=row+","+str(ssr[0])+","+str(ssr[1])+","+str(ssr[2])
  if len(row)<10:
    continue
  f.write(row+"\n")

# plot
  c.pop(-1)
  c.insert(0,b)
  try:
    d=np.array(c)
  except:
    continue
  e=d.T
  plt.figure(100)
  plt.clf()
  plt.title("Tc")
  tl = [0] * 10
  hd = []
  for i in range(0,10):
    tl[i], = plt.plot(x,e[i],label="T"+str(i+1))
  for i in range(0,10):
    hd.append(tl[i])
    plt.legend(handles=hd)
  plt.pause(0.1)
#
  plt.figure(200)
  plt.clf()
  tl2=[0] * 8
  hd2=[]
  plt.title("ADC")
  for i in range(0,8):
    tl2[i], = plt.plot(x,e[i+20],label="ADC"+str(i+1))
  for i in range(0,8):
    hd2.append(tl2[i])
    plt.legend(handles=hd2)
  plt.pause(0.1)
#
 except KeyboardInterrupt:
  print("KeyboardInterrupt:")
  GPIO.output(11, False)
  GPIO.output(12, False)
  GPIO.output(13, False)
  GPIO.output(15, 1)    # pump keeps running
  GPIO.output(16, False)  
  GPIO.output(18, False)
  GPIO.output(19, False)
  ser.close()
  f.close()
  exit()