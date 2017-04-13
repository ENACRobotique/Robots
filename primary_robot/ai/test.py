import communication

comm = communication.Communication()
msgDown = communication.sMessageDown()

type_int = input("Msg type ?")
msgDown.message_type = communication.eTypeDown(type_int)
print("Sending...")
comm.send_message(msgDown)
print("Sent !")