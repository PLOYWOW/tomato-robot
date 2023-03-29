#!/usr/bin/env python
import rospy
import numpy as np
import cv2

class GUI():
    def __init__(self,img_name,type_name,param_name_list,max_param,MIN,MAX):
    #'RS_SETTING','dist[cm]',['DEPTH'],np.array([500]),np.load('./' + rs_img_name + type_name + 'min.npy'),np.load('./' + rs_img_name + type_name + 'max.npy')
        self.img_name = str(img_name)
        self.type_name = str(type_name)
        self.argnum = len(param_name_list) #depth_param_name_list = ['DEPTH'], len() = 1
        self.param_name_list = param_name_list 
        self.max_param = np.array([180,255,255])
        self.min = MIN
        self.max = MAX
        # print("MIN = ",end="")
        # print(MIN)
        # print("MAX = ",end="")
        # print(MAX)
        ############## Initial HSV value ####
        ### mask1
        self.mask1_H_min = 0
        self.mask1_S_min = 0
        self.mask1_V_min = 0
        self.mask1_H_max = 19
        self.mask1_S_max = 255
        self.mask1_V_max = 255
        ### mask2
        self.mask2_H_min = 133
        self.mask2_S_min = 0
        self.mask2_V_min = 0
        self.mask2_H_max = 180
        self.mask2_S_max = 255
        self.mask2_V_max = 255
        #####################################
    
    def get_param_as_tuple(self): #used to remove background
        return tuple(self.min),tuple(self.max)
    
    def changeColor(self,val):
        # print("IN change color callback fn")
        for i in range(self.argnum): #1
            # print("     IN for of change color callback fn")
            # print("IN change color, i = ",end="")
            # print(i)
    
            self.min[i] = int(cv2.getTrackbarPos(self.type_name + self.param_name_list[i] + '_min', self.img_name))
            self.max[i] = int(cv2.getTrackbarPos(self.type_name + self.param_name_list[i] + '_max', self.img_name))
            
            #print("2 self.min[i] = ",end="")
            #print(i,self.min[i])

            #print("2 self.max[i] = ",end="")
            #print(i,self.max[i])
            
        #self.save_track_parameter()

    def create_trackbar(self):
        # print("IN create trackbar fn")
        background = np.zeros((10,300,3), np.uint8)
        cv2.namedWindow(self.img_name) #'RS_SETTING'
        #create trackbar
        for i in range(self.argnum): #1
            # print("     IN for of create trackbar fn")
            # print("####################")
            # print("current min dist is "+str(self.min[i]))
            # print("current max dist is "+str(self.max[i]))
            # print("####################")
            # if i == 0:
            #     print("H")
            # elif i == 1:
            #     print("S")
            # elif i == 2:
            #     print("V")
            print("self.type_name",self.type_name)
            if self.type_name == "mask1--":
                self.min[0] = self.mask1_H_min # H_min
                # print("FINAL>>>"+str(self.min[0]))
                self.min[1] = self.mask1_S_min # S_min
                # print("FINAL>>>"+str(self.min[1]))
                self.min[2] = self.mask1_V_min # V_min
                # print("FINAL>>>"+str(self.min[2]))
                self.max[0] = self.mask1_H_max # H_max
                # print("FINAL>>>"+str(self.max[0]))
                self.max[1] = self.mask1_S_max # S_max
                # print("FINAL>>>"+str(self.max[1]))
                self.max[2] = self.mask1_V_max # V_max
                # print("FINAL>>>"+str(self.max[2]))
            elif self.type_name == "mask2--":
                self.min[0] = self.mask2_H_min # H_min
                #print("FINAL>>>"+str(self.min[0]))
                self.min[1] = self.mask2_S_min # S_min
                # print("FINAL>>>"+str(self.min[1]))
                self.min[2] = self.mask2_V_min # V_min
                # print("FINAL>>>"+str(self.min[2]))
                self.max[0] = self.mask2_H_max # H_max
                print("FINAL>>>"+str(self.max[0]),i)
                self.max[1] = self.mask2_S_max # S_max
                # print("FINAL>>>"+str(self.max[1]))
                self.max[2] = self.mask2_V_max # V_max
                # print("FINAL>>>"+str(self.max[2]))
            #make instance to create trackbar  
            # print("IN create_trackbar, ",end="")
            
            # print("i = "+str(i))
            print("self.min[i] = ",end="")
            print(i,self.min[i])

            print("self.max[i] = ",end="")
            print(i,self.max[i])
            
            cv2.createTrackbar(self.type_name + self.param_name_list[i] + '_min', self.img_name, self.min[i], self.max_param[i], self.changeColor)

            cv2.createTrackbar(self.type_name + self.param_name_list[i] + '_max', self.img_name, self.max[i], self.max_param[i], self.changeColor)
            # print("self.max[i] = ",end="")
            # print(self.max[i])
            #if (self.type_name == "mask2--") and (i == 0) and (self.max[i] == -1):
                # print("YESSSSSSSSSSSSSSSSSSSSSS!!!!!!!!!!!!!!!!!!")
                #self.max[i] == self.mask2_H_max
                # print("!!!!!!!!self.mask2_H_max = "+str(self.mask2_H_max))
                # print("!!self.max[i] = "+str(self.max[i]))
            
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        # cv2.imshow(self.img_name,background) #'RS_SETTING',np.zeros((10,300,3), np.uint8)
        
    def save_track_parameter(self): #sed to save previous set depth dist
        # print("*******IN save track parameter*******")
        np.save('./' + self.img_name + self.type_name + 'min.npy',self.min)
        np.save('./' + self.img_name + self.type_name + 'max.npy',self.max)