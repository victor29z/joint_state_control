#ifndef JOINT_DATA_TYPE_H
#define JOINT_DATA_TYPE_H
typedef struct{
    unsigned int joint_pos_raw[16];
    unsigned int joint_pos_abs[16];
    unsigned int joint_force[16];
    bool joint_pos_valid[16];
    bool joint_online[16];
    unsigned char keyvalue[16];

}JOINT_DAT_TYPE;

typedef struct{
    struct{
        unsigned int joystick_lr;
        unsigned int joystick_ud;
        bool top_key1;
        bool top_key2;
        bool top_key3;
        bool buttom_key;
        bool enable_key;

    }left,right;
}HANDLE_DAT_TYPE;

#define KEY_MASK_TOPKEY1    0x01
#define KEY_MASK_TOPKEY2    0x02
#define KEY_MASK_TOPKEY3    0x04
#define KEY_MASK_BUTKEY    0x10
#define KEY_MASK_ENABLEKEY    0x08
#define JOYSTICK_LR_MUSK       0x0fff
#define JOYSTICK_UD_MUSK       0xfff000
#endif // JOINT_DATA_TYPE_H
