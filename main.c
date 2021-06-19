

/*=== PROJECT:  JBC station prototype based on DI-1 C245 ====
	authr:    GEORGI CHAKAROV, 
    	location: Technical University branch Plovdiv, Bulgaria
	version:  v0, 
	date:     2021
*/

#include <stdint.h>
typedef enum { false, true } bool;
#define FACTOR       0.42  // mV * factor = degree 'C

// PIN IO CONFIGURATION  
#define PIN_BUTTON_UP            D4
#define PIN_BUTTON_OK            D2
#define PIN_BUTTON_DOWN          D3
#define PIN_ZUMMER               D9

/*  
 *  === MEMORY ALLOCATION TABLE ===	
		 
 *  EEPROM: 
		 -----------
		|_param_val_|
		|   250     |  // p.0 [ C ] LEVEL_TEMP1
		|   300	    |  // p.1 [ C ] LEVEL_TEMP2
		|   350     |  // p.2 [ C ] LEVEL_TEMP3
		|   400     |  // p.3 [ C ] LIMIT_TEMP_MAX
		|   150     |  // p.4 [ C ] LIMIT_TEMP_MIN
		 -----------
		 -----------------------------------------------------------------------------------------
		|_res_|___name__|_SP__|_SLEEP_TEMP_|_SLEEP_DELAY_|_HIBERN_DELAY_|_Kp_|_Ki_|_Kd_|_TEMP_adj_|
		|  -1 | "T210"  | 0   |     0      |     0       |      0       |  0 |  0 |  0 |    0     |  // tool 0 T210
		| 460 | "T245"  | 300 |     180    |     2       |      15      |  0 |  0 |  0 |    0     |  // tool 1 T245 configured
		|  -1 | "T470"  | 0   |     0      |     0       |      0       |  0 |  0 |  0 |    0     |  // tool 2 T470
		|  -1 | "PA120" | 0   |     0      |     0       |      0       |  0 |  0 |  0 |    0     |  // tool 3 PA120
		|  -1 | "HT420" | 0   |     0      |     0       |      0       |  0 |  0 |  0 |    0     |  // tool 4 HT420
		|  -1 | "DS360" | 0   |     0      |     0       |      0       |  0 |  0 |  0 |    0     |  // tool 5 DS360
		|  -1 | "DR560" | 0   |     0      |     0       |      0       |  0 |  0 |  0 |    0     |  // tool 6 DR560
		|  -1 | "AP130" | 0   |     0      |     0       |      0       |  0 |  0 |  0 |    0     |  // tool 7 AP130	
		 -----------------------------------------------------------------------------------------
		 

  * RAM MEMORY:
		 ---------|-------------------------------------------------------------------------- 
		|__addrs__|_CV__|_PV__|_flag_sleep_|_flag_hibern_|_Le_|_Lt_|_LRe_|_twp_|_addrs_param_|
 --------> 	| 0x3224  |  0  |  24 |    false   |    false    |  0 |  0 |  0  |  0  |    0x42C8   | 
|		|---------|-----------------------------------------------------------------------------------------|																											-
|  		|	  |_res_|___name__|_SP__|_SLEEP_TEMP_|_SLEEP_DELAY_|_HIBERN_DELAY_|_Kp_|_Ki_|_Kd_|_TEMP_adj_|
|		| 0x42C8  | 460 |  "T245" | 300 |     180    |     2       |      15      |  0 |  0 |  0 |    0     |  // *var saved in eeprom
|		|---------------------------------------------------------------------------------------------------|
|		
| * PORT[] table:                     
|		 ---------------|----------------- const ---------------- 
|		|indx| tool_ptr | PIN_ADC | PIN_PWM | PIN_ID | PIN_STAND |
 ------<<       |  0 | 	0x3224  |   A0    |    D5   |   D6   |    D7     |
		|---------------|----------------------------------------|



*/

// DEFAULT VALUE PARAMETERS STATION
uint16_t default_parameters[5] = {
	250,  // p.0 [ C ] LEVEL_TEMP1
	300,  // p.1 [ C ] LEVEL_TEMP2
	350,  // p.2 [ C ] LEVEL_TEMP3
	400,  // p.3 [ C ] LIMIT_TEMP_MAX
	150,  // p.4 [ C ] LIMIT_TEMP_MIN
};


typedef struct{
	// settings tool saved in EEPROM 
	uint8_t    res               ;  // ADC value on connected tool ID ressistor [0=1023]
	char       name[6]           ;  // name  ex:"T245"
	uint16_t   SP                ;  // SetPoint  [TEMP]
	uint16_t   SLEEP_TEMP        ;  // SP_Sleep  [TEMP]
	int16_t    SLEEP_DELAY       ;  // -1 OFF, 0 - 900s
	int8_t     HIBERNATION_DELAY ;  // -1 OFF, 0 - 30min 
	float      Kp, Ki, Kd        ;  // coefficients tuning PID
	float      TEMP_adj          ;  // +- 7.00'C  ;  
}STOOL;

typedef struct{
	uint8_t  CV                  ;  // Control Value [0-99%]
	double   PV                  ;  // Process Value [TEMP]
	
	bool     flag_sleep          ;
	bool     flag_hibern         ;
	
	double   LAST_e              ;  // last error
	long     LAST_t              ;  // last time
	long     LAST_Re             ;  // Riemann sum; error*time
	
	long     time_when_placed    ;
	STOOL    param               ;  // ptr to parameters - tool
}TOOL;

typedef struct{
	uint8_t  PIN_ADC_INPUT       ;  // GPIO's
	uint8_t  PIN_PWM_OUTPUT      ; 
	uint8_t  PIN_SLEEP_TRIG      ; 
	uint8_t  PIN_ID_INPUT        ;
	ID       tool_id             ; // -1 = null, no tool set
}PORT;

/* === DEFINE PORT's I/O map === */
int PORT_n = 1;
PORT port[PORT_n] = {
	// FOR port_ 1:
	{
		.PIN_ADC_INPUT           = A0   , 
		.PIN_PWM_OUTPUT          = D5   , 
		.PIN_SLEEP_TRIG          = D7   ,
		.PIN_ID_INPUT            = D6   
	}
}


	
int main(){
	
	/* CONFIRM I/O REGISTER ;;  
	 * SET PWM output pin to zero;;
	 * DISPLAY SHOW "STARTUP":
		 ---------------------------      
		|	      JBC           |
		|---------------------------|
		|      www.jbctools.com     |   
		|   	                    |
		|  			    |				
		| Loading profiles ...      |				
		 ---------------------------	
	*/
	long time_start = millis();
	DISPLAY_show_startup();
	
	// LOAD PARAMETERS STATION
	uint8_t  parameters_cell_num = sizeof(default_parameters)/sizeof(uint16_t);
	uint16_t parameters[ parameters_cell_num ];                                        // allocation work parameters array
	uint8_t  ptr_buffer* = EEPROMreadCell( 0, sizeof(uint16_t)*parameters_cell_num );  // read cell from EEPROM 
	memcpy( &parameters, (uint16_t*) ptr_buffer, sizeof(uint16_t) );                   // copy data from buffer to parameters ;; cast uint8_t byte array to uint16_t
	
	
	// AWAIT 
	while( time_start + 180000 > millis() ) ; // display show 3s "startup"
	
	
	// MAIN CICLE
	uint8_t i = 0;
	volatile bool menu_open = false;
	while( menu_open ) { // PORT[0] , PORT[i]
		res = analogRead( PORT[i].PIN_ADC );
	
		if( res > 1020 ) { // disconnected
		
			display("no connected tool");
			if(  flag_first ) {
				// off power iron
				digitalWrite( PORT[i].PIN_PWM_OUTPUT, 0 ); 
				// free memory allocation - obj tool, stool
				free( PORT[i].tool_ptr );
				free( PORT[i].tool_ptr.addrs_param );
			}
		
		} else {
			if( flag_first ) {
				// new connection:
				STOOL *ptr_stool_param  = readFromEEPROM_where_res_value( res );
				// if not is null
				TOOL tool;  							// new empty tool obj
				tool.address_param = ptr_stool_param;   // set ptr to stool obj in tool obj
				PORT[i].tool_ptr = &tool; 				// set ptr to tool in port obj
			}

			// CHECK  Solder iron placed on Stand: sleep/hibernation 
			if( digitalRead( PORT[i].PIN_SLEEP_TRIG ) == 0 ){ 
				long time_now = millis();
				long sleep_time = (PORT[i].tool_ptr.param.SLEEP_DELAY*60000); // const sleep time [ms];
				if( PORT[i].tool_ptr.time_when_placed = 0 ) PORT[i].tool_ptr.time_when_placed = time_now; 
				if( PORT[i].tool_ptr.time_when_placed > (sleep_time - time_now) ){  
				
					// Sleep one time
					if( !PORT[i].tool_ptr.flag_sleep ) 
						PORT[i].tool_ptr.flag_sleep = !PORT[i].tool_ptr.flag_sleep;
				
					if( PORT[i].tool_ptr.time_when_placed > (sleep_time + PORT[i].tool_ptr.param.HIBERNATION_DELAY) - time_now)){
					
						// Hibernation one time
						if( !PORT[i].tool_ptr.hibern_flag ) 
							PORT[i].tool_ptr.hibern_flag = !PORT[i].tool_ptr.hibern_flag;
					}
				}
			}
			else{
				// ;; reset ON
				PORT[i].tool_ptr.time_when_placed = 0; 
				PORT[i].tool_ptr.flag_sleep  = false;
				PORT[i].tool_ptr.hibern_flag = false;
			}
		
		
			// READ ADC to PV['C]
			PORT[i].tool_ptr.PV = ( FACTOR * analogRead( PORT[i].PIN_ADC_INPUT )) + PORT[i].tool_ptr.param.TEMP_adj ; 
			// Calculate CV reaction
			Calculate_Reaction( PORT[i].tool_ptr );  // PID 
			// Latch new CV on port 
			digitalWrite( PORT[i].PIN_PWM_OUTPUT, PORT[i].tool_ptr.CV ); 
			DISPLAY_show_TOOLWORK( PORT[i].tool_ptr ); 
		
		}
	
		i++;
		if( i = 1 ) i = 0;
	
	}
		
	
	// tools off
	for( uint8_t indx = 0; indx < PORT_n; indx++) {
		digitalWrite( PORT[indx].PIN_PWM_OUTPUT, 0 );
	}
	
	while( DISPLAY_menu() ); // open menu 
	
	// restart with new configuration;
	asm volatile ("jmp 0x7800");
}

void Calculate_Reaction( TOOL *tool ){  // calculation with pid 
	double e  = (( tool.flag_hibern || tool.flag_no_tool || tool.flag_error )?
		(0): ( 
			( tools[n].flag_sleep)? ( tools[n].param.SLEEP_TEMP ) : (tools[n].param.SP)
		)) - tool.PV ; 
	long   time_now  = millis();
	long   delta_t   = time_now - tool.LAST_t;
	double delta_e   = e - tool.LAST_e ; 
	
	tool.LAST_Re   += e*delta_t; // sum add e(t)
	
        // result calculate 0-100%
	tool.CV = map((tool.param.Kp*e + tool.LAST_Re*tool.param.Ki + (delta_e/delta_t)*tool.param.Kd), -limit, +limit, 0, 100 )
	
	// update value
	tool.LAST_t   = time_now;
	tool.LAST_e   = e;
	
}

uint8_t* EEPROMreadCell( uint16_t address, uint16_t cell_n ){ 
	uint8_t _buffer_byte_array[ cell_n ];
	for( uint16_t n = 0; n < cell_n; n++){   
		_buffer_byte_array[n] = EEPROM.read(address + n);  // 1byte read
	}
	return _buffer_byte_array;
}

int8_t menu_index = 0;
int8_t cursor[1] = { 0 }; // L0
bool DISPLAY_MENU(){
	// TODO menu option ;;
	return true;
}

void DISPLAY_show_TOOLWORK( TOOL *tool){
	/*
		 --------------------------      
		|       280  300  350      |
		|--------------------------|
		|	SP: [300]          |   +-5 tolerance set 
		|	PV: 298            | 
		|			   |
		|   Power 3%|____|_____|   |
		 --------------------------
	*/
	// WORK dislay;; 
	// SLEEP display;;
	// HIBER display;;
}

void DISPLAY_show_startup(){
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.setFontRefHeightExtendedText();
	u8g2.setFontPosTop();
	u8g2.setFontDirection(0);
	u8g2.setDrawColor(1);
	u8g2.drawStr( 54, 10,  "JBC");
	u8g2.setFont(u8g2_font_6x12_tf);
	u8g2.drawStr( 16, 25,  "www.jbctools.com");  // 96
	u8g2.setFont(u8g2_font_5x7_tf);
	u8g2.drawStr( 1, 56,  "Loading ... ");  // 96
	u8g2.sendBuffer();
}



