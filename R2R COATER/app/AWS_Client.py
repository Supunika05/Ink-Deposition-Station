from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

AWS_IOT_ENDPOINT = "a2bfjm7oo4305c-ats.iot.ap-south-1.amazonaws.com"
CLIENT_ID        = "R2R_LaptopThing02"
PATH_ROOT_CA     = "R2R_LaptopThing2/AmazonRootCA1.pem"
PATH_CERT        = "R2R_LaptopThing2/certificate.pem.crt"
PATH_PRIVATE_KEY = "R2R_LaptopThing2/private.pem.key"

client = AWSIoTMQTTClient(CLIENT_ID)
client.configureEndpoint(AWS_IOT_ENDPOINT, 8883)
client.configureCredentials(PATH_ROOT_CA, PATH_PRIVATE_KEY, PATH_CERT)
client.configureOfflinePublishQueueing(-1)
client.configureDrainingFrequency(2)
client.configureConnectDisconnectTimeout(10)
client.configureMQTTOperationTimeout(5)

try:
    client.connect()
    print("Connected to AWS IoT")
except Exception as e:
    print(f"Error: Failed to connect to AWS IoT - {e}")

topic_cam = "laptop/control/camera"
topic_command = "laptop/control/command"

topic_video = "esp32_cam/stream"