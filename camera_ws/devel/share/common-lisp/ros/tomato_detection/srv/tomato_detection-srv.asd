
(cl:in-package :asdf)

(defsystem "tomato_detection-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SelectTomato" :depends-on ("_package_SelectTomato"))
    (:file "_package_SelectTomato" :depends-on ("_package"))
    (:file "switch_cam" :depends-on ("_package_switch_cam"))
    (:file "_package_switch_cam" :depends-on ("_package"))
  ))