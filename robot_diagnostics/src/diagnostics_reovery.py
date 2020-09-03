#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
from monitoring_msgs.msg import KeyValues
from threading import Thread, Lock
import time
import roslaunch
import threading

mutex = Lock()
class DiagnosticsRecovery:

    def __init__(self):
        print "Constructor Diagnostics Recovery"
        self.number_subscriber = rospy.Subscriber("/critical_errors", KeyValues, self.criticalMessageCallback)
        self.m_diagnosticsRecovery  = rospy.get_param("/diagnosticsRecovery")
        self.currentRecovery  = None
      
        

        '''paramSize = len(self.m_diagnosticsRecovery)
        for i in range(0,paramSize):
            print self.m_diagnosticsRecovery[i]["node"]'''

    def criticalMessageCallback(self, msg): 
        msgSize = len(msg.keyvalues)
        for i in range(msgSize):
          keyValue = msg.keyvalues[i].key
          self.criticalErrorClassification(keyValue)
          
    def criticalErrorClassification(self,keyValue): 
        #print keyValue
        key   = keyValue.split(':')[0] 
        key   = key[1:]
        value = keyValue.split(':')[1]
        self.recoveryProcedure(key)

    def recoveryProcedure(self,nodeName): 

        mutex.acquire()
        print("mutex is now locked")

        paramSize = len(self.m_diagnosticsRecovery)
        for i in range(0,paramSize):
            
            if nodeName == self.m_diagnosticsRecovery[i]["node"]:
                print "Hereeee"
                self.currentRecovery =  self.m_diagnosticsRecovery[i]

                


        mutex.release()
        print("mutex is now unlocked")

    def getRecovery(self):
        print "I am here too"
        return self.currentRecovery

    def testFun(self):
        package = 'robot_diagnostics'
        executable = 'dummyBattery'
        node = roslaunch.core.Node(package, executable)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
    def print_cube(num): 
        print num
        

if __name__ == '__main__':
    rospy.init_node('diagnostics_recoverys')
    dia = DiagnosticsRecovery()
    
    r = rospy.Rate(10)
    t1 = threading.Thread(target=dia.print_cube, args=(10)) 
    #t1.join()
    # starting thread 1 
    t1.start() 
    while not rospy.is_shutdown():
        x = dia.getRecovery()
        print x
        r.sleep()
        rospy.spin()