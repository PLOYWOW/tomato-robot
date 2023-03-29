;; Auto-generated. Do not edit!


(when (boundp 'handeye_cam::Msg)
  (if (not (find-package "HANDEYE_CAM"))
    (make-package "HANDEYE_CAM"))
  (shadow 'Msg (find-package "HANDEYE_CAM")))
(unless (find-package "HANDEYE_CAM::MSG")
  (make-package "HANDEYE_CAM::MSG"))

(in-package "ROS")
;;//! \htmlinclude Msg.msg.html


(defclass handeye_cam::Msg
  :super ros::object
  :slots (_w _h _num ))

(defmethod handeye_cam::Msg
  (:init
   (&key
    ((:w __w) 0)
    ((:h __h) 0)
    ((:num __num) 0)
    )
   (send-super :init)
   (setq _w (round __w))
   (setq _h (round __h))
   (setq _num (round __num))
   self)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:h
   (&optional __h)
   (if __h (setq _h __h)) _h)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:serialization-length
   ()
   (+
    ;; int32 _w
    4
    ;; int32 _h
    4
    ;; int32 _num
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _w
       (write-long _w s)
     ;; int32 _h
       (write-long _h s)
     ;; int32 _num
       (write-long _num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _w
     (setq _w (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _h
     (setq _h (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num
     (setq _num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get handeye_cam::Msg :md5sum-) "e7443ea74a07cdd552dd4948fd2619cf")
(setf (get handeye_cam::Msg :datatype-) "handeye_cam/Msg")
(setf (get handeye_cam::Msg :definition-)
      "int32 w
int32 h
int32 num

")



(provide :handeye_cam/Msg "e7443ea74a07cdd552dd4948fd2619cf")


