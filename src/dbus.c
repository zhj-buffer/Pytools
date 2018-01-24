#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dbus/dbus-glib.h>
#include <dbus/dbus.h>
#include <unistd.h>

#define D_VDBUS_NAME    "com.pi.videoRecognitionConnect"
#define D_VDBUS_PATH   "/com/pi/videoRecognitionService"
#define D_VDBUS_IFACE   "com.pi.videoRecognitionInterface"

int init_session_bus(DBusConnection **session_bus,
                     const char *connect_name)
{
	DBusError        err;
	int              ret;

	dbus_error_init(&err);
	*session_bus = dbus_bus_get(DBUS_BUS_SYSTEM, &err);
	if ((dbus_error_is_set(&err)) || (NULL == *session_bus)) {
		fprintf(stderr, "Connection Error (%s)\n", err.message);
		dbus_error_free(&err);
		return -1;
	}
	printf("Max received size: %ld\n", dbus_connection_get_max_received_size(*session_bus));
	printf("Max message size: %ld\n", dbus_connection_get_max_message_size(*session_bus));
        printf("get session bus @ %p\n", *session_bus);
	ret = dbus_bus_request_name(*session_bus, connect_name, DBUS_NAME_FLAG_REPLACE_EXISTING, &err);
	if (dbus_error_is_set(&err) || (DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER != ret)) {
		fprintf(stderr, "Name Error (%s)\n", err.message);
		dbus_error_free(&err);
		return -1;
	}

	return 0;
}

int method_arg_func(DBusMessage *msg, void *param) {
    DBusMessageIter arg;
    char            *strparam = (char *)param;

    if(msg == NULL)
        return -1;

    if(param != NULL) {
        dbus_message_iter_init_append(msg, &arg);
        if(!dbus_message_iter_append_basic(&arg, DBUS_TYPE_STRING, &strparam)) {
            printf("Out of Memory\n");
            return -2;
        }
    }
    return 0;
}

int method_call(DBusConnection *connection, 
                const char     *func, 
                int            (*argfunc)(DBusMessage *msg, void *param), 
                void           *param) {
    DBusError     err;
    DBusMessage   *msg;
    dbus_uint32_t serial =0;

    dbus_error_init(&err);
    msg = dbus_message_new_method_call(D_VDBUS_NAME, D_VDBUS_PATH, D_VDBUS_IFACE, func);
    if(msg == NULL){
        printf("Message is NULL\n");
        return -1;
    }

    if(argfunc != NULL) {
        if(argfunc(msg, param) < 0) {
            printf("set param of message faili\n");
            return -2;
        }
    }

    if(!dbus_connection_send(connection, msg, &serial)){
    	printf("Out of Memory\n");
        dbus_message_unref(msg);
        return -3;
    }

    dbus_connection_flush(connection);
    dbus_message_unref(msg);

    return 0;
}

void camera_without_replay(DBusConnection * connection, char *mesg) {
    //printf("<<<<<<<<<<<<<<<<< Send data >>>>>>>>>>>>>>>>>>>>>>>>>>");
    method_call(connection, "receive_data", method_arg_func, mesg);
}


/*
 Send signal to target
 session_dbus: D-Bbus Session
 target_service_path:
 target_interface_name:
 signal_name:
 contents: 
*/
int send_signal_to_target(DBusConnection *session_bus,
	                  const char *target_service_path,
                      const char *target_interface_name,
                      const char *signal_name,
	                  char *contents)
{
   DBusMessage      *msg;
   DBusMessageIter  iter;
   int              ret;
   dbus_uint32_t    serial = 0;

   //printf("Sending signal with value: %s\n", contents);
   msg = dbus_message_new_signal(target_service_path, target_interface_name, signal_name);
   //printf("%s, msg: %p\n", __func__, msg);
   if (NULL == msg) {
      fprintf(stderr, "Message Null\n");
      return -1;
   }
   dbus_message_iter_init_append(msg, &iter);
   ret = dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &contents);
   if (!ret) {
      fprintf(stderr, "Out Of Memory!\n");
      return -1;
   }
   ret = dbus_connection_send(session_bus, msg, &serial);
   if (!ret) {
      fprintf(stderr, "Out Of Memory!\n");
      return -1;
   }

   dbus_connection_flush(session_bus);
   dbus_message_unref(msg);

   return 0;
}

int emit_event_signal(DBusConnection *conn, char *event_name)
{
    printf("==============>>>>>\n");
    if (send_signal_to_target(conn,
		    "/com/pi/cameraSensorService",
		    "com.pi.Event.emit",
		    "event_emit_signal", event_name)) {
        return -1;
    }
    return 0;
}

int send_object_messages(DBusConnection *session_bus, char *contents)
{
    if(send_signal_to_target(session_bus, 
                             "/com/pi/cameraSensorService", 
                             "com.pi.videoRecognitionInterface",
                             "object_messages",
                             contents)){
        printf("Send Object Message to Video Recognition Service Fail!");
        return -1;
	}

	return 0;
}

int send_control_command(DBusConnection *session_bus, char *contents)
{
    //printf("move:=************************** %s\n", contents);
    if(send_signal_to_target(session_bus, 
                             "/com/pi/cameraSensorService", 
                             "com.pi.deviceSensorInterface",
                             "motor_control",
                             contents)){
        printf("Send Motor Control Command to Device Sensor Service Fail!");
        return -1;
	}

    return 0;
}

