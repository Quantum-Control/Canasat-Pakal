import  smbus
import time
import math

#CONSTANTS FOR INITIALIZAION:

L3G2000D_CTRL_REG1 = 0x20
L3G2000D_CTRL_REG2 = 0x21
L3G2000D_CTRL_REG3 = 0x22
L3G2000D_CTRL_REG4 = 0x23
L3G2000D_CTRL_REG5 = 0x24

L3G2000D_REG_OUT_TEMP = 0x26
L3G2000D_STATUS_REG= 0x27

L3G2000D_REG_OUT_X_L=0x28  
L3G2000D_REG_OUT_X_H=0x29
L3G2000D_REG_OUT_Y_L=0x2A
L3G2000D_REG_OUT_Y_H=0x2B
L3G2000D_REG_OUT_Z_L=0x2C
L3G2000D_REG_OUT_Z_H=0x2D

L3G2000D_FIFO_CTRL_REG=0x2E
L3G2000D_FIFO_SRC_REG=0x2F

L3G2000D_INT1_CFG=0x30
L3G2000D_INT1_SRC=0x31
L3G2000D_INT1_SH_XH =0x32
L3G2000D_INT1_TSH_XL=0x33
L3G2000D_INT1_TSH_YH=0x34
L3G2000D_INT1_TSH_YL=0x35
L3G2000D_INT1_TSH_ZH=0x36
L3G2000D_INT1_TSH_ZL=0x37

L3G2000D_INT1_DURATION= 0x38
L3G2000D_ADDRESS=0x69

bus=smbus.SMBus(1)

class L3G4200D():
    
    def __init__(self):
        
        self.useCalibrate=False
        self.dAxis=[0,0,0]
        self.thresholdAxis=[0,0,0]
        self.t=[0,0,0]
        self.actualthreshold=0
        
        self.GyroRaw=[0,0,0]
        self.GyroUnits=[0,0,0]
        self.__data_config_()
        
    def __data_config_(self):
       
        bus.write_byte_data(L3G2000D_ADDRESS,L3G2000D_CTRL_REG1,0x0F)
        bus.write_byte_data(L3G2000D_ADDRESS,L3G2000D_CTRL_REG4,L3G2000D_INT1_CFG)
       
       
    def __read_raw(self):
       
        Xvalue= [bus.read_byte_data(L3G2000D_ADDRESS,L3G2000D_REG_OUT_X_L),bus.read_byte_data(L3G2000D_ADDRESS,L3G2000D_REG_OUT_X_H)]
        Yvalue= [bus.read_byte_data(L3G2000D_ADDRESS,L3G2000D_REG_OUT_Y_L),bus.read_byte_data(L3G2000D_ADDRESS,L3G2000D_REG_OUT_Y_H)]
        Zvalue= [bus.read_byte_data(L3G2000D_ADDRESS,L3G2000D_REG_OUT_Z_L),bus.read_byte_data(L3G2000D_ADDRESS,L3G2000D_REG_OUT_Z_H)]
        
        
        self.GyroRaw[0]= Xvalue[1]<<8| Xvalue[0]
        self.GyroRaw[1]= Yvalue[1]<<8| Yvalue[0]
        self.GyroRaw[2]= Zvalue[1]<<8| Zvalue[0]
        
        if self.GyroRaw[0]>32767:
            self.GyroRaw[0]-=65536
            
        if self.GyroRaw[1]>32767:
            self.GyroRaw[1]-=65536
        
        if self.GyroRaw[2]>32767:
            self.GyroRaw[2]-=65536
    
    def Get_Raw(self):
        return self.GyroRaw[0],self.GyroRaw[1],self.GyroRaw[2]
        
        
    def Get_value(self):
        return self.GyroUnits[0],self.GyroUnits[1],self.GyroUnits[2]
    def read_value(self):
        
        
        self.__read_raw()
        
        if self.useCalibrate==True:
            self.GyroUnits[0]=(self.GyroRaw[0]-self.dAxis[0])*0.070
            self.GyroUnits[1]=(self.GyroRaw[1]-self.dAxis[1])*0.070
            self.GyroUnits[2]=(self.GyroRaw[2]-self.dAxis[2])*0.070
            
        else:
            self.GyroUnits[0]=(self.GyroRaw[0])*0.070
            self.GyroUnits[1]=(self.GyroRaw[1])*0.070
            self.GyroUnits[2]=(self.GyroRaw[2])*0.070
     
     
        if self.actualthreshold >0:
            
            if abs(self.GyroUnits[0])< self.t[0]:
                self.GyroUnits[0]=0
                
            if abs(self.GyroUnits[1])< self.t[1]:
                self.GyroUnits[1]=0
            
            if abs(self.GyroUnits[2])< self.t[2]:
                self.GyroUnits[2]=0
    
    def Calibrate(self,Samples):
        useCalibrate = True
        
        sumX=0
        sumY=0
        sumZ=0
        
        sigmaX=0
        sigmaY=0
        sigmaZ=0
        
        for i in range(Samples):
            self.__read_raw()
            sumX+=self.GyroRaw[0]
            sumY+=self.GyroRaw[1]
            sumZ+=self.GyroRaw[2]
            
            sigmaX+=self.GyroRaw[0]*self.GyroRaw[0]
            sigmaY+=self.GyroRaw[1]*self.GyroRaw[1]
            sigmaZ+=self.GyroRaw[2]*self.GyroRaw[2]
            
        self.dAxis[0]=sumX/Samples
        self.dAxis[1]=sumY/Samples
        self.dAxis[2]=sumZ/Samples
        
        self.thresholdAxis[0]=math.sqrt((sigmaX/Samples)-(self.dAxis[0]*self.dAxis[0]))
        self.thresholdAxis[1]=math.sqrt((sigmaX/Samples)-(self.dAxis[1]*self.dAxis[1]))
        self.thresholdAxis[2]=math.sqrt((sigmaX/Samples)-(self.dAxis[2]*self.dAxis[2]))
        
        if self.actualthreshold  >0:
            self.setThreshold(self.actualthreshold)
        
    def Set_Threshold(self,multiple):
        
        if multiple>0:
            if self.useCalibrate==False:
                self.Calibrate()
            
            self.t[0]=self.threshold[0]*multiple
            self.t[1]=self.threshold[1]*multiple
            self.t[2]=self.threshold[2]*multiple
            
        else:
                
            self.t[0]=0
            self.t[1]=0
            self.t[2]=0
                
             
        self.actualthreshold=multiple
