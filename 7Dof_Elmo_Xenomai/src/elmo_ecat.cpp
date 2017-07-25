#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "elmo_ecat.h"

/*********************static variables start*********************/
uint8_t elmo_ecat::cyctime=10;

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
    cntlwd_val(0x80),
    modop_val(0),
    tartor_val(0),
    name("")
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
    //printf("elmo %d\n",MotorPos1);
    ec_domain_state_t ds;

    ecrt_domain_state(domainRxPdo, &ds);

    if (ds.working_counter != domainRxPdo_state.working_counter)
        printf("elmo %d: domainRxPdo: WC %u.\n", MotorPos1, ds.working_counter);
    if (ds.wc_state != domainRxPdo_state.wc_state)
        printf("elmo %d: domainRxPdo: State %u.\n", MotorPos1, ds.wc_state);

    domainRxPdo_state = ds;

    ecrt_domain_state(domainTxPdo, &ds);

    if (ds.working_counter != domainTxPdo_state.working_counter)
        printf("elmo %d: domainTxPdo: WC %u.\n", MotorPos1, ds.working_counter);
    if (ds.wc_state != domainTxPdo_state.wc_state)
        printf("elmo %d: domainTxPdo: State %u.\n", MotorPos1, ds.wc_state);

    domainTxPdo_state = ds;
}

void elmo_ecat::check_master_state()
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

void elmo_ecat::check_slave_config_states(void)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc_motor, &s);

    if (s.al_state != sc_motor_state.al_state)
        printf("elmo %d: Motor: State 0x%02X.\n", MotorPos1, s.al_state);
    if (s.online != sc_motor_state.online)
        printf("elmo %d: Motor: %s.\n", MotorPos1, s.online ? "online" : "offline");
    if (s.operational != sc_motor_state.operational)
        printf("elmo %d: Motor: %soperational.\n",MotorPos1,
               s.operational ? "" : "Not ");

    sc_motor_state = s;
}

int elmo_ecat::sdo_write_u8(uint16_t index, uint8_t subindex, uint8_t val)
{
    fprintf(stderr, "Creating SDO requests...\n");
    if (!(sdo_write_u8_request = ecrt_slave_config_create_sdo_request(sc_motor, index, subindex, 1)))
    {
        fprintf(stderr, "Failed to create SDO %x request.\n",index);
        return -1;
    }
    ecrt_sdo_request_timeout(sdo_write_u8_request, 500); // ms  500

    bool isWriteModeOk=false;
    while(1){
       usleep(1000*50);
        ecrt_master_receive(master);
        EC_WRITE_U8(ecrt_sdo_request_data(sdo_write_u8_request), val);
        switch (ecrt_sdo_request_state(sdo_write_u8_request)) {
        case EC_REQUEST_UNUSED: // request was not used yet
            fprintf(stderr, "Request was not used yet!\n");
            ecrt_sdo_request_write(sdo_write_u8_request);
            break;
        case EC_REQUEST_BUSY:
            fprintf(stderr, "Still busy...\n");
            break;
        case EC_REQUEST_SUCCESS:
            fprintf(stderr, "SDO 0x%x:%x write OK!\n",index, subindex);
            isWriteModeOk = true;
            break;
        case EC_REQUEST_ERROR:
            fprintf(stderr, "Failed to read SDO!\n");
            break;
        }
        if(isWriteModeOk){
            break;
        }
    }
    return 0;
}
/*********************private fun end*********************/

/*********************public fun end*********************/
void elmo_ecat::setname(const std::string& _name)
{
    name=_name;
}

const std::string& elmo_ecat::getname() const
{
    return name;
}

int elmo_ecat::config_pdos()
{
    // Configuring PDOs
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_motor, EC_END, elmo_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domainRxPdo, domainRxPdo_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domainTxPdo, domainTxPdo_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }
    return 0;
}

int elmo_ecat::link_pdos()
{
    if (!(domainRxPdo_pd = ecrt_domain_data(domainRxPdo))) {
        fprintf(stderr, "Link RxPdo domain failed!\n");
        return -1;
    }

    if (!(domainTxPdo_pd = ecrt_domain_data(domainTxPdo))) {
        fprintf(stderr, "Link TxPdo domain failed!\n");
        return -1;
    }
    return 0;
}

void elmo_ecat::sync_pdos()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domainRxPdo);
    ecrt_domain_process(domainTxPdo);

    // check process data state (optional)
    // check_domain_state();

    //if (!(cycle_counter % (1000/cyctime))) {                            // do this at 1 Hz
    // check for master state (optional)
    //check_master_state();
    // check for islave configuration state(s) (optional)
    //check_slave_config_states();
    //}

    //position limition
    if(status_val==4663)
    {
        if(tarpos_val>maxpos) tarpos_val=maxpos;
        if(tarpos_val<minpos) tarpos_val=minpos;
        //error=true;
        //errorid=1234;
    }

    EC_WRITE_S32(domainRxPdo_pd + elmo_tarpos, tarpos_val);
    EC_WRITE_U32(domainRxPdo_pd + elmo_digout, digout_val);
    EC_WRITE_U16(domainRxPdo_pd + elmo_cntlwd, cntlwd_val);
    EC_WRITE_S8(domainRxPdo_pd + elmo_modop, modop_val);
    EC_WRITE_S16(domainRxPdo_pd + elmo_tartor, tartor_val);

    actpos_val=EC_READ_S32(domainTxPdo_pd + elmo_actpos);
    digin_val=EC_READ_S32(domainTxPdo_pd + elmo_digin);
    status_val=EC_READ_U16(domainTxPdo_pd + elmo_status);
    disop_val=EC_READ_S8(domainTxPdo_pd + elmo_disop);
    acttor_val=EC_READ_S8(domainTxPdo_pd + elmo_acttor);

    // send process data
    ecrt_domain_queue(domainRxPdo);
    ecrt_domain_queue(domainTxPdo);
    ecrt_master_send(master);
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
