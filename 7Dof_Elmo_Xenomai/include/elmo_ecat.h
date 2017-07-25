#ifndef __ELMO_ECAT_H__
#define __ELMO_ECAT_H__


#include <string>
#include "ecrt.h"

#define ELMO_GOLD 0x0000009A, 0x00030924
#define ENC 524288  //2^19

class elmo_ecat
{
public:
    elmo_ecat(ec_master_t *_master,
              uint16_t _MotorPos0,
              uint16_t _MotorPos1,
              uint8_t _cyctime);
    ~elmo_ecat();

public:
    void setname(const std::string& _name) ;
    void getname(const std::string& _name) ;
    const std::string& getname() const ;

    int config_pdos();
    int link_pdos();
    void sync_pdos( );

    void tarpos(int32_t);
    void digout(uint32_t);
    void cntlwd(uint16_t);
    void modop(int8_t);
    void tartor(int16_t);

    int32_t actpos(void);
    uint32_t digin(void);
    uint16_t status(void);
    int8_t disop(void);
    int16_t acttor(void);

    void tarpos_modulo(double);    //angle
    double actpos_modulo(void);    //angle
    void tarpos_bias_modulo(double);    //angle
    double actpos_bias_modulo(void);    //angle

    void setbias(int32_t);
    void setmaxpos(int32_t);
    void setminpos(int32_t);
    void setbias_module(double);
    void setmaxpos_module(double);
    void setminpos_module(double);

private:
    int sdo_write_u8(uint16_t, uint8_t,uint8_t);
    void check_domain_state(void);
    void check_master_state(void);
    void check_slave_config_states(void);

private:
    std::string name;

    static uint8_t cyctime;

    int32_t tarpos_val;                     //S32
    uint32_t    digout_val;                //U32
    uint16_t cntlwd_val;                  //U16
    int8_t modop_val;                     //S8
    int16_t tartor_val;                //S16

    int32_t actpos_val;                                 //S32
    uint32_t digin_val;                   //U32
    uint16_t status_val;       //U16
    int8_t disop_val;                               //S8
    int16_t acttor_val;                //S16

    uint16_t MotorPos0;
    uint16_t MotorPos1;

//    double tarpos_modulo;                     //angle
//    double actpos_modulo;                     //angle

    unsigned int elmo_tarpos;    //vS32
    unsigned int elmo_digout;    //U32
    unsigned int elmo_cntlwd;    //U16
    unsigned int elmo_modop;   //S8
    unsigned int elmo_tartor;    //S16

    unsigned int elmo_actpos;    //S32
    unsigned int elmo_digin;       //S32
    unsigned int elmo_status;     //U16
    unsigned int elmo_disop;      //S8
    unsigned int elmo_acttor;      //S16

    ec_master_t *master=NULL;
    ec_master_state_t master_state;

    ec_domain_t *domainRxPdo=NULL;
    ec_domain_t *domainTxPdo=NULL;
    ec_domain_state_t domainRxPdo_state;
    ec_domain_state_t domainTxPdo_state;

    ec_slave_config_t *sc_motor=NULL;
    ec_slave_config_state_t sc_motor_state;

    uint8_t *domainRxPdo_pd=NULL;
    uint8_t *domainTxPdo_pd=NULL;

    ec_pdo_entry_reg_t domainRxPdo_regs[6];
    ec_pdo_entry_reg_t domainTxPdo_regs[6];

    static ec_pdo_entry_info_t elmo_pdo_entries_RxPdo_0[3];
    static ec_pdo_entry_info_t elmo_pdo_entries_RxPdo_1[1];
    static ec_pdo_entry_info_t elmo_pdo_entries_RxPdo_2[1];
    static ec_pdo_entry_info_t elmo_pdo_entries_TxPdo_0[3];
    static ec_pdo_entry_info_t elmo_pdo_entries_TxPdo_1[1];
    static ec_pdo_entry_info_t elmo_pdo_entries_TxPdo_2[1];

    static ec_pdo_info_t elmo_pdo_RxPdo[3];
    static ec_pdo_info_t elmo_pdo_TxPdo[3];
    static ec_sync_info_t elmo_syncs[5];

    ec_sdo_request_t *sdo_write_u8_request=NULL;

    int32_t bias;
    int32_t maxpos;
    int32_t minpos;
};

#endif
