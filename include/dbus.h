#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dbus/dbus-glib.h>
#include <dbus/dbus.h>
#include <unistd.h>

#define D_DBUS_NAME "com.example.service"
#define D_DBUS_PATH "/com/example/someobject"
#define D_DBUS_IFACE "com.example.interface"

int init_session_bus(DBusConnection **session_bus, const char *connect_name);
void send_a_method_call(DBusConnection * connection, char * param);
void two_value_test(DBusConnection * connection);
void hello_world_without_replay(DBusConnection * connection);
void camera_without_replay(DBusConnection * connection, char *mesg);
int send_object_messages(DBusConnection *session_bus, char *contents);
int send_control_command(DBusConnection *session_bus, char *contents);
int emit_event_signal(DBusConnection *session_bus, char *contents);