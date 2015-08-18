
(cl:in-package :asdf)

(defsystem "marvel_v_0_1-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "qr_kill" :depends-on ("_package_qr_kill"))
    (:file "_package_qr_kill" :depends-on ("_package"))
  ))