;; Auto-generated. Do not edit!


(when (boundp 'tomato_detection::switch_cam)
  (if (not (find-package "TOMATO_DETECTION"))
    (make-package "TOMATO_DETECTION"))
  (shadow 'switch_cam (find-package "TOMATO_DETECTION")))
(unless (find-package "TOMATO_DETECTION::SWITCH_CAM")
  (make-package "TOMATO_DETECTION::SWITCH_CAM"))
(unless (find-package "TOMATO_DETECTION::SWITCH_CAMREQUEST")
  (make-package "TOMATO_DETECTION::SWITCH_CAMREQUEST"))
(unless (find-package "TOMATO_DETECTION::SWITCH_CAMRESPONSE")
  (make-package "TOMATO_DETECTION::SWITCH_CAMRESPONSE"))

(in-package "ROS")





(defclass tomato_detection::switch_camRequest
  :super ros::object
  :slots (_req_status ))

(defmethod tomato_detection::switch_camRequest
  (:init
   (&key
    ((:req_status __req_status) nil)
    )
   (send-super :init)
   (setq _req_status __req_status)
   self)
  (:req_status
   (&optional (__req_status :null))
   (if (not (eq __req_status :null)) (setq _req_status __req_status)) _req_status)
  (:serialization-length
   ()
   (+
    ;; bool _req_status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _req_status
       (if _req_status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _req_status
     (setq _req_status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tomato_detection::switch_camResponse
  :super ros::object
  :slots (_data ))

(defmethod tomato_detection::switch_camResponse
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; uint8 _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass tomato_detection::switch_cam
  :super ros::object
  :slots ())

(setf (get tomato_detection::switch_cam :md5sum-) "c1d0f6b5d3ccfee714fe1fdff0074c75")
(setf (get tomato_detection::switch_cam :datatype-) "tomato_detection/switch_cam")
(setf (get tomato_detection::switch_cam :request) tomato_detection::switch_camRequest)
(setf (get tomato_detection::switch_cam :response) tomato_detection::switch_camResponse)

(defmethod tomato_detection::switch_camRequest
  (:response () (instance tomato_detection::switch_camResponse :init)))

(setf (get tomato_detection::switch_camRequest :md5sum-) "c1d0f6b5d3ccfee714fe1fdff0074c75")
(setf (get tomato_detection::switch_camRequest :datatype-) "tomato_detection/switch_camRequest")
(setf (get tomato_detection::switch_camRequest :definition-)
      "bool req_status
---
uint8 data
")

(setf (get tomato_detection::switch_camResponse :md5sum-) "c1d0f6b5d3ccfee714fe1fdff0074c75")
(setf (get tomato_detection::switch_camResponse :datatype-) "tomato_detection/switch_camResponse")
(setf (get tomato_detection::switch_camResponse :definition-)
      "bool req_status
---
uint8 data
")



(provide :tomato_detection/switch_cam "c1d0f6b5d3ccfee714fe1fdff0074c75")


