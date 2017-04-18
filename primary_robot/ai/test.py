import communication

comm = communication.Communication()
msgDown = communication.sMessageDown()
while 1:
    type_int = input("Msg type ?")
    msgDown.message_type = communication.eTypeDown(int(type_int))
    print("Sending...")
    comm.send_message(msgDown)
    print("Sent !")