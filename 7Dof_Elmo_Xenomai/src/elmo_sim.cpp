#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "elmo_ecat.h"

/*********************static variables start*********************/
uint8_t elmo_ecat::cyctime=4;

//output1 1600
ec_pdo_entry_info_t elmo_ecat::elmo_pdo_entries_RxPdo_0[] = {
    { 0x607A, 0x00, 32 },     //Target Position
    { 0x60FE, 0x01, 32 },     //Digital Outputs
    { 0x6040, 0x00, 16 },     //Control word
};
//output1 160B
ec_pdo_entry_info_t elmo_ecat::elmo_pdo_entries_RxPdo_1[] = {
    { 0x6060, 0x00, 8 },     //Mode of operation
};
//output2 160C
ec_pdo_entry_info_t elmo_ecat::elmo_pdo_entries_RxPdo_2[] = {
    { 0x6071, 0x00, 16 },     //Target Torque
};

//input 1A00
ec_pdo_entry_info_t elmo_ecat::elmo_pdo_entries_TxPdo_0[] = {
    { 0x6064, 0x00, 32 },     //Position actual value
    { 0x60FD, 0x00, 32 },     //Digital Inputs
    { 0x6041, 0x00, 16 },     //Status word
};
//input 1A0B
ec_pdo_entry_info_t elmo_ecat::elmo_pdo_entries_TxPdo_1[] = {
    { 0x6061, 0x00, 8 },     //Mode of operation display
};
//input 1A13
ec_pdo_entry_info_t elmo_ecat::elmo_pdo_entries_TxPdo_2[] = {
    { 0x6077, 0x00, 16 },     //Target Torque
};

ec_pdo_info_t elmo_ecat::elmo_pdo_RxPdo[] = {
    { 0x1600, 3, elmo_pdo_entries_RxPdo_0 },
    { 0x160B, 1, elmo_pdo_entries_RxPdo_1 },
    { 0x160C, 1, elmo_pdo_entries_RxPdo_2 },
};

ec_pdo_info_t elmo_ecat::elmo_pdo_TxPdo[] = {
    { 0x1A00, 3, elmo_pdo_entries_TxPdo_0 },
    { 0x1A0B, 1, elmo_pdo_entries_TxPdo_1 },
    { 0x1A13, 1, elmo_pdo_entries_TxPdo_2 },
};

ec_sync_info_t elmo_ecat::elmo_syncs[] = {
    { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_ENABLE },
    { 1, EC_DIR_INPUT, 0, NULL, EC_WD_ENABLE },
    { 2, EC_DIR_OUTPUT, 3, elmo_pdo_RxPdo, EC_WD_ENABLE },
    { 3, EC_DIR_INPUT, 3, elmo_pdo_TxPdo, EC_WD_ENABLE },
    { 0xff }
};
/*********************static variables end*********************/


elmo_ecat::elmo_ecat(ec_master_t *_master, uint16_t _MotorPos0, uint16_t _MotorPos1, uint8_t _cyctime):
    tarpos_val(0),
    digout_val(0),
    cntlwd_val(0),
    modop_val(0),
    tartor_val(0)
{
    master=_master;
    MotorPos0=_MotorPos0;
    MotorPos1=_MotorPos1;
    cyctime=_cyctime;

    ec_pdo_entry_reg_t domainRxPdo_regs_temp[6] = {
        { MotorPos0,MotorPos1,ELMO_GOLD, 0x607A, 0x00, &elmo_tarpos },
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x60FE, 0x01, &elmo_digout},
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x6040, 0x00, &elmo_cntlwd },
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x6060, 0x00, &elmo_modop },
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x6071, 0x00, &elmo_tartor },
        {}
    };

    ec_pdo_entry_reg_t domainTxPdo_regs_temp[6] = {
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x6064, 0x00, &elmo_actpos },
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x60FD, 0x00, &elmo_digin },
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x6041, 0x00, &elmo_status },
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x6061, 0x00, &elmo_disop },
        { MotorPos0,MotorPos1, ELMO_GOLD, 0x6077, 0x00, &elmo_acttor },
        {}
    };

    memcpy(domainRxPdo_regs, domainRxPdo_regs_temp,sizeof(domainRxPdo_regs));
    memcpy(domainTxPdo_regs,domainTxPdo_regs_temp,sizeof(domainTxPdo_regs));

    // Creates a new process data domain
    domainRxPdo = ecrt_master_create_domain(master);
    if (!domainRxPdo)
        printf("Create RxPdo domain failed.\n");
    domainTxPdo = ecrt_master_create_domain(master);
    if (!domainTxPdo)
        printf("Create TxPdo domain failed.\n");

    // Obtains a slave configuration
    if (!(sc_motor = ecrt_master_slave_config(master, MotorPos0, MotorPos1, ELMO_GOLD)))
        fprintf(stderr, "Failed to get slave configuration.\n");

    //startup sdo
    sdo_write_u8(0x60C2,01,cyctime);  //cycle time
    sdo_write_u8(0x6060,0,8);  //modes of operation
}

elmo_ecat::~elmo_ecat()
{
    //delete sdo_opmode_write;
    //delete sdo_cycletime_write;
}


/*********************private fun start*********************/
void elmo_ecat::check_domain_state(void)
{
}

void elmo_ecat::check_master_state()
{
}

void elmo_ecat::check_slave_config_states(void)
{
}

int elmo_ecat::sdo_write_u8(uint16_t index, uint8_t subindex, uint8_t val)
{
    return 0;
}
/*********************private fun end*********************/


int elmo_ecat::config_pdos()
{
    return 0;
}

int elmo_ecat::link_pdos()
{
    return 0;
}

void elmo_ecat::sync_pdos()
{
    //position limition
    if(status_val==4663)
    {
        if(tarpos_val>maxpos) tarpos_val=maxpos;
        if(tarpos_val<minpos) tarpos_val=minpos;
        //error=true;
        //errorid=1234;
    }

    actpos_val=tarpos_val;
    digin_val=digout_val;
    switch (cntlwd_val) {
    case 0x80:
        status_val=592;
        break;
    case 0x06:
        status_val=561;
        break;
    case 0x07:
        status_val=563;
        break;
    case 0x0F:
        status_val=4663;
        break;
    case 0x1F:
        status_val=4663;
        break;
    default:
        break;
    }
    disop_val=modop_val;
    acttor_val=tartor_val;
}

void elmo_ecat::tarpos(int32_t _tarpos)
{
    tarpos_val=_tarpos;
}

void elmo_ecat::digout(uint32_t _digout)
{
    digout_val=_digout;
}

void elmo_ecat::cntlwd(uint16_t _cntlwd)
{
    cntlwd_val=_cntlwd;
}

void elmo_ecat::modop(int8_t _modop)
{
    modop_val=_modop;
}

void elmo_ecat::tartor(int16_t _tartor)
{
    tartor_val=_tartor;
}

int32_t elmo_ecat::actpos(void)
{
    return actpos_val;
}

uint32_t elmo_ecat::digin(void)
{
    return digin_val;
}

uint16_t elmo_ecat::status(void)
{
    return status_val;
}

int8_t elmo_ecat::disop(void)
{
    return disop_val;
}

int16_t elmo_ecat::acttor(void)
{
    return acttor_val;
}

void elmo_ecat::tarpos_modulo(double _tarpos_modulo)    //angle
{
    tarpos_val=(int32_t)(_tarpos_modulo*ENC/360.0);
}

double elmo_ecat::actpos_modulo(void)    //angle
{
    return actpos_val*360.0/ENC;
}

void elmo_ecat::tarpos_bias_modulo(double _tarpos_bias_modulo)    //angle
{
    tarpos_val=(int32_t)(_tarpos_bias_modulo*ENC/360.0)+bias;
}

double elmo_ecat::actpos_bias_modulo(void)    //angle
{
    return (actpos_val-bias)*360.0/ENC;
}

void elmo_ecat::setbias(int32_t _bias)
{
    bias=_bias;
}

void elmo_ecat::setmaxpos(int32_t _maxpos)
{
    maxpos=bias+_maxpos;
}

void elmo_ecat::setminpos(int32_t _minpos)
{
    minpos=bias+_minpos;
}

void elmo_ecat::setbias_module(double _bias)
{
    bias=(int32_t)(_bias*ENC/360.0);
}

void elmo_ecat::setmaxpos_module(double _maxpos)
{
    maxpos=bias+(int32_t)(_maxpos*ENC/360.0);
}

void elmo_ecat::setminpos_module(double _minpos)
{
    minpos=bias+(int32_t)(_minpos*ENC/360.0);
}
