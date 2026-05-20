/******************************************
 * PID CASCADE + QUINTIC S-CURVE 
 * (ĐÃ SỬA: DÙNG THƯ VIỆN ENCODER.H VÀ XUNG X4 - KHÔNG DÙNG KALMAN FILTER)
 * Full sketch with motor_enabled state
 * - Outer position PID: 50 Hz (20 ms)
 * - Inner velocity PID: 200 Hz (5 ms)
 * - Commands: SET a b c | HOME | HUT | THA 
 * - Đã loại bỏ hoàn toàn cơ chế bù trừ xung khi đảo chiều (anti-backlash).
 * * ĐÃ KHẮC PHỤC LỖI STRAY CHARACTER ('\302', '\240')
 ******************************************/

// BẮT BUỘC: Thêm thư viện Encoder.h
#include <Encoder.h>
#include <math.h> // Để dùng các hàm toán học như sin, copysign, fabs

// Các hằng số cho phép tính toán (cần thiết cho joint_angle_rad)
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329251994329576923690768489 // M_PI / 180
#endif

// Arduino Mega hoặc Mega 2560 PINOUT
const int M1_IN1 = 14;
const int M1_IN2 = 15;
const int M1_ENA = 4;
const int ENC1_A = 3; 	 // INT0
const int ENC1_B = 2; 	 // INT1

const int M2_IN1 = 17;
const int M2_IN2 = 12;
const int M2_ENA = 5;
const int ENC2_A = 21; // INT5
const int ENC2_B = 20; // INT4

const int M3_IN1 = 11;
const int M3_IN2 = 10;
const int M3_ENA = 9;
const int ENC3_A = 18; // INT3
const int ENC3_B = 19; // INT2

const int ENA = 7; 	 	 // control for solenoid / vacuum
const int CTHT_1 = 46;
const int CTHT_2 = 44;
const int CTHT_3 = 42;

/********* Khai báo Encoder (SỬ DỤNG THƯ VIỆN) **********/
// Tham số: (Chân B, Chân A) để tương thích với thư viện Encoder.h khi chân A là chân ngắt
Encoder enc1_lib(ENC1_B, ENC1_A); 
Encoder enc2_lib(ENC2_B, ENC2_A);
Encoder enc3_lib(ENC3_B, ENC3_A);


/********* Encoder Count **********/
long enc1 = 0; 
long enc2 = 0;
long enc3 = 0;

/********* Velocity Calculation **********/
long prev_enc1 = 0;
long prev_enc2 = 0;
long prev_enc3 = 0;
float current_velocity1 = 0; // xung / ms (Tính toán trực tiếp)
float current_velocity2 = 0;
float current_velocity3 = 0;

/********* CASCADE PID (Hệ số đã được điều chỉnh) **********/
const int STABLE_COUNT = 500; // Số lần ổn định liên tiếp để kết thúc lệnh

// Position PID gains (outer)
float Kp_pos[] = {0, 8.5, 7.5, 7.5}; 
float Ki_pos[] = {0, 0.0, 0.0, 	0.0};
float Kd_pos[] = {0, 0.8, 0.1, 	0.15}; 

// Velocity PID gains (inner)
float Kp_vel[] = {0, 6.3, 6.8, 8.5}; 
float Ki_vel[] = {0, 0.2, 0.1 ,1.8}; 
float Kd_vel[] = {0, 0.0, 0.0, 0.0}; 

float Kff_v[] = {0, 0,3.1, 6.1}; 
float Kff_a[] = {0, 0.0,0.01, 0.03}; 
float Kgrav[] = {0, 0, 2.1, 1.5};

int last_dir[4] = {0,0,0,0}; // Vẫn giữ lại biến này nhưng không dùng trong PID
float vel_lpf[4] = {0, 0, 0, 0};
// integrators and previous errors
float i_pos[4] = {0,0,0,0}, p_pos[4] = {0,0,0,0};
float i_vel[4] = {0,0,0,0}, p_vel[4] = {0,0,0,0};

// LPF for PWM output smoothing
float prev_out[4] = {0,0,0,0};
const float alpha = 0.6; // Hệ số LPF cho PWM 

// PWM limits
float pwm_limit = 180.0; 
const float PWM_DEADZONE = 4; // Không dùng

/********* Timing **********/
const unsigned long VEL_INTERVAL_MS = 5; 	// 200 Hz
const unsigned long POS_INTERVAL_MS = 20; 	// 50 Hz
unsigned long lastVel = 0;
unsigned long lastPos = 0;

/********* Encoder -> degree (mech config) - XUNG X4 **********/
const float Hesotruyen = 1;
// M1: (16 * 270 * (60/20) / 1) * 4
const float pulsesPerRevolution1 = (16.0 * 270.0 * (60.0 / (20.0 * Hesotruyen))) * 4.0; 
const float pulsesPerDegree1 = pulsesPerRevolution1 / 360.0;
// M2: 16 * 270 * 4
const float pulsesPerRevolution2 = 16.0 * 506.0 * 4.0; 
const float pulsesPerDegree2 = pulsesPerRevolution2 / 360.0;
// M3: 11 * 90 * 4
const float pulsesPerRevolution3 = 11.0 * 90.0 * 4.0; 
const float pulsesPerDegree3 = pulsesPerRevolution3 / 360.0;

/********* Trajectory (quintic S-curve) **********/
const float MAX_VELOCITY = 5.0; 	// xung / ms
const float MIN_TRAJ_TIME =1.5; 	 // s (min)
float acoef[4][6]; // Hệ số quỹ đạo bậc 5
float traj_T[4] = {0,0,0,0};
unsigned long traj_t0_ms[4] = {0,0,0,0};
bool traj_active[4] = {false,false,false,false};

float start_position[4] = {0,0,0,0};
float target_position[4] = {0,0,0,0};

float s_curve_target_pos[4] = {0,0,0,0};
float s_curve_target_vel[4] = {0,0,0,0}; // xung / ms
float s_curve_target_acc[4] = {0,0,0,0}; // xung / ms^2

float traj_vel[4]; 	// vận tốc từ S-curve (không dùng trực tiếp, chỉ để debug)
float pid_vel_cmd[4]; // vận tốc PID hiệu chỉnh

/********* Control / state **********/
long final_target_pulse[4] = {0,0,0,0};
long pulse[4] = {0,0,0,0};
String inputString = "";
bool stringComplete = false;
bool start_homing[4] = {0,0,0,0};
long nStable[4] = {0,0,0,0};
bool jobDoneArr[4] = {true,true,true,true}; 
const int allowError1 = 3;
const int allowError2 = 3;
const int allowError3 = 3;
bool has_moved = false;

// NEW: motor enabled state
bool motor_enabled = false;
bool is_all_done = false;

/********* Prototypes **********/
float joint_angle_rad(int m);
void update_encoder_readings();
void calculate_current_velocity(); 
void plan_quintic_trajectory(int m);
void sample_trajectory(int m);
void positionPID(int m);
void velocityPID(int m);
void applyPWM(int m, float pwm);
void resetPIDState();
void brakeAll();
void stopAll();
void startHomingSequence();
void printlog();


/******************************************
 Motor helper functions 
******************************************/
void fwd1(int pwm) { digitalWrite(M1_IN1, LOW); 	digitalWrite(M1_IN2, HIGH); analogWrite(M1_ENA, pwm); }
void rev1(int pwm) { digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW); 	analogWrite(M1_ENA, pwm); }
void stop1() 	 	 { digitalWrite(M1_IN1, LOW); 	digitalWrite(M1_IN2, LOW); 	analogWrite(M1_ENA, 0); }
void brake1() 	 { digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, HIGH); analogWrite(M1_ENA, 255); }

void fwd2(int pwm) { digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH); 	analogWrite(M2_ENA, pwm); }
void rev2(int pwm) { digitalWrite(M2_IN1, HIGH); 	digitalWrite(M2_IN2, LOW); analogWrite(M2_ENA, pwm); }
void stop2() 	 	 { digitalWrite(M2_IN1, LOW); 	digitalWrite(M2_IN2, LOW); 	analogWrite(M2_ENA, 0); }
void brake2() 	 { digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, HIGH); analogWrite(M2_ENA, 255); }

void fwd3(int pwm) { digitalWrite(M3_IN1, LOW); 	digitalWrite(M3_IN2, HIGH); analogWrite(M3_ENA, pwm); }
void rev3(int pwm) { digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW); 	analogWrite(M3_ENA, pwm); }
void stop3() 	 	 { digitalWrite(M3_IN1, LOW); 	digitalWrite(M3_IN2, LOW); 	analogWrite(M3_ENA, 0); }
void brake3() 	 { digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, HIGH); analogWrite(M3_ENA, 255); }


/******************************************
 Cập nhật giá trị Encoder (ĐỌC TỪ THƯ VIỆN)
 ******************************************/
void update_encoder_readings() {
	noInterrupts();
	enc1 = enc1_lib.read();
	enc2 = enc2_lib.read();
	enc3 = enc3_lib.read();
	interrupts();
}

/******************************************
 calculate velocities (xung / ms) - TÍNH TRỰC TIẾP
 ******************************************/
void calculate_current_velocity() {
	// Tính vận tốc dựa trên sự thay đổi encoder trong khoảng thời gian VEL_INTERVAL_MS
	current_velocity1 = (float)(enc1 - prev_enc1) / (float)VEL_INTERVAL_MS;
	current_velocity2 = (float)(enc2 - prev_enc2) / (float)VEL_INTERVAL_MS;
	current_velocity3 = (float)(enc3 - prev_enc3) / (float)VEL_INTERVAL_MS;

	prev_enc1 = enc1;
	prev_enc2 = enc2;
	prev_enc3 = enc3;
}

/******************************************
 Quintic trajectory planner 
 ******************************************/
void plan_quintic_trajectory(int m) {
	float p0 = start_position[m];
	float pf = target_position[m];
	float delta = pf - p0;
	float dist = fabs(delta);

	float T = MIN_TRAJ_TIME;
	if (dist > 0.0) {
		float max_vel_s = MAX_VELOCITY * 1000.0; // Chuyển sang xung / giây
		float T_calc = 2.0 * dist / max_vel_s; // s
		T = T_calc;
	}
	if (T < MIN_TRAJ_TIME) T = MIN_TRAJ_TIME;

	float T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
	// Các hệ số của quỹ đạo bậc 5: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
	acoef[m][0] = p0;
	acoef[m][1] = 0.0; 
	acoef[m][2] = 0.0; 
	acoef[m][3] = (10.0*(pf - p0)) / T3;
	acoef[m][4] = (-15.0*(pf - p0)) / T4;
	acoef[m][5] = (6.0*(pf - p0)) / T5;

	traj_T[m] = T;
	traj_t0_ms[m] = millis();
	traj_active[m] = true;
}

/******************************************
 sample trajectory
 ******************************************/
void sample_trajectory(int m) {
	if (!traj_active[m]) {
		s_curve_target_pos[m] = target_position[m];
		s_curve_target_vel[m] = 0.0;
		s_curve_target_acc[m] = 0.0;
		return;
	}
	unsigned long tnow_ms = millis();
	float t_rel = (tnow_ms - traj_t0_ms[m]) / 1000.0; // Chuyển sang giây
	float T = traj_T[m];

	if (t_rel >= T) {
		s_curve_target_pos[m] = target_position[m];
		s_curve_target_vel[m] = 0.0;
		s_curve_target_acc[m] = 0.0;
		traj_active[m] = false;
		return;
	}
	float t = t_rel;
	float t2 = t*t, t3 = t2*t, t4 = t3*t;
	
	// Tính vị trí
	float pos = acoef[m][0] + acoef[m][1]*t + acoef[m][2]*t2 + acoef[m][3]*t3 + acoef[m][4]*t4 + acoef[m][5]*t2*t3;
	// Tính vận tốc (xung / giây)
	float vel_s = acoef[m][1] + 2.0*acoef[m][2]*t + 3.0*acoef[m][3]*t2 + 4.0*acoef[m][4]*t3 + 5.0*acoef[m][5]*t4; 
	float vel_ms = vel_s / 1000.0; // Chuyển sang xung / ms
 	// Gia tốc (xung / s^2)
	float acc_s2 =2.0 * acoef[m][2]+ 6.0 * acoef[m][3] * t+ 12.0 * acoef[m][4] * t2+ 20.0 * acoef[m][5] * t3;
	// đổi sang xung / ms^2
	s_curve_target_acc[m] = acc_s2 / 1e6;
	s_curve_target_pos[m] = pos;
	traj_vel[m] = vel_ms;
	s_curve_target_vel[m] = vel_ms; // Cập nhật target velocity từ quỹ đạo

}

/******************************************
 Position PID (outer) -> produce target velocity
 ******************************************/
void positionPID(int m) {
	long current_enc = (m==1)?enc1 : (m==2?enc2:enc3);
	float target_pos = s_curve_target_pos[m];
	float e_pos = target_pos - (float)current_enc;
	
	// ===== I TERM (ANTI-WINDUP NHẸ) =====
	const float KI_POS_DEADZONE = 3.0;
	if (fabs(e_pos) > KI_POS_DEADZONE)
		i_pos[m] += e_pos * (POS_INTERVAL_MS / 1000.0);
	else
		i_pos[m] *= 0.95;

	i_pos[m] = constrain(i_pos[m], -20000.0, 20000.0);

	// ===== D TERM =====
	float d_pos = (e_pos - p_pos[m]) / (POS_INTERVAL_MS / 1000.0);
	p_pos[m] = e_pos;

	float v_target;

	// ===== FIX GÓC NHỎ – KHỚP 2 =====
	if (m == 2 && fabs(e_pos) < 20.0) {   // ~10 độ
		float vmin = 0.8;
		v_target = Kp_pos[m] * e_pos;

		if (fabs(v_target) < vmin)
			v_target = copysign(vmin, v_target);
	}
	else {
		v_target = Kp_pos[m] * e_pos
				 + Ki_pos[m] * i_pos[m]
				 + Kd_pos[m] * d_pos;
	}

	// ===== GIỚI HẠN VẬN TỐC =====
	float vmax;
	if (m == 2 && s_curve_target_vel[m] < 0)
		vmax = MAX_VELOCITY * 0.2;
	else if (s_curve_target_vel[m] < 0)
		vmax = MAX_VELOCITY * 0.35;
	else
		vmax = MAX_VELOCITY;

	v_target = constrain(v_target, -vmax, vmax);

	pid_vel_cmd[m] = v_target;
}

/******************************************
 Velocity PID (inner) -> produce PWM
 ******************************************/
void velocityPID(int m) {
	// ===== 1. CURRENT VELOCITY (Vòng lặp trong 5ms) =====
	float raw_vel;
	if (m == 1) raw_vel = current_velocity1;
	else if (m == 2) raw_vel = current_velocity2;
	else raw_vel = current_velocity3;

	// ===== 2. LPF VELOCITY (Lọc nhiễu cho vận tốc đọc được) =====
	// Hệ số 0.7 cho M2 là hợp lý nhất.
	if (m == 1) vel_lpf[m] = 0.5 * vel_lpf[m] + 0.5 * raw_vel;
	else if (m == 3) vel_lpf[m] = 0.6 * vel_lpf[m] + 0.4 * raw_vel;
	else vel_lpf[m] = 0.7 * vel_lpf[m] + 0.3 * raw_vel;

	float current_vel = vel_lpf[m];
	float v_target = pid_vel_cmd[m]; // Vận tốc mục tiêu từ Vòng Position PID
	float e_vel = v_target - current_vel;
    float out = 0.0;
    
	// Lỗi vị trí cuối cùng (dùng cho các hiệu chỉnh gần đích)
	long enc = (m==1?enc1:(m==2?enc2:enc3));
	long posErr = final_target_pulse[m] - enc;

	// =================================================================
	// ===== 3. PID CORE (Bù sai số) =====
	// =================================================================
	
	// I TERM (Anti-Windup)
	const float KI_VEL_DEADZONE = 0.3; // Tăng nhẹ để I-term chỉ hoạt động khi sai số lớn hơn
	if (fabs(e_vel) > KI_VEL_DEADZONE)
		i_vel[m] += e_vel * (VEL_INTERVAL_MS / 1000.0);
	else
		i_vel[m] *= 0.9; // Giảm nhẹ khi ở Deadzone
	
	i_vel[m] = constrain(i_vel[m], -1000.0, 1000.0);

	// D TERM
	float d_vel = (e_vel - p_vel[m]) / (VEL_INTERVAL_MS / 1000.0);
	p_vel[m] = e_vel;

	// Tính toán PID
	float pid_out =
		  Kp_vel[m] * e_vel
		+ Ki_vel[m] * i_vel[m]
		+ Kd_vel[m] * d_vel;

	// =================================================================
	// ===== 4. FEEDFORWARD (Lực tiên đoán) =====
	// =================================================================
	float FF_v = Kff_v[m] * v_target;
	float FF_a = Kff_a[m] * s_curve_target_acc[m];

	// Giảm gia tốc FF khi đi xuống (để tránh phản ứng quá mức)
	if (v_target < 0) {
		float v_ratio = fabs(v_target) / MAX_VELOCITY;
		v_ratio = constrain(v_ratio, 0.0, 1.0);
		FF_a *= v_ratio * 0.3;
	}
    
	// Giảm nhẹ FF khi rất gần đích (tránh trượt)
	if (abs(posErr) < 8) {
		FF_v = 0;
		FF_a = 0;
	}

	// =================================================================
	// ===== 5. GRAVITY FF (Bù trọng lực) =====
	// =================================================================
	float grav = 0.0;
	if (m == 2 || m == 3) {
		float theta = joint_angle_rad(m);
		grav = Kgrav[m] * sin(theta);

		// Giảm Grav FF cho M3 khi M2 xoay nhanh (giả định M2 gây rung/nhiễu)
		if (m == 3) {
			float coupling = constrain(fabs(current_velocity2) / 2.0, 0.0, 0.8);
			grav *= (1.0 - coupling);
		}
        
        // Trọng lực luôn được cộng vào nếu nó chống lại trọng lực hoặc trừ đi nếu nó giúp trọng lực
		if (posErr < 0) {
            // Khi đi xuống, grav (dương) sẽ làm tăng tốc. Ta cần trừ nó ra.
            grav = -grav;
        }}


	// =================================================================
	// ===== 6. TỔNG OUTPUT TRƯỚC HIỆU CHỈNH DESCENT/SCALE =====
	// =================================================================
	out = pid_out + FF_v + FF_a + grav;
    
	
    // =================================================================
    // ===== 7. DESCENT / SCALE MODE (Hiệu chỉnh tùy biến) =====
    // =================================================================
    
    // **Hiệu chỉnh chung khi đi xuống (Descent)**
    if ((m == 2 || m == 3) && v_target < 0) {
        // Giảm hệ số PID/FF khi đi xuống để tránh phản ứng thái quá do trọng lực
        // Tỷ lệ này sẽ giúp motor chạy mượt mà hơn khi đi xuống
        out *= 0.7; // Giảm tổng tác động 30%
        
        // Giới hạn cứng cho Descent Mode
        float descent_limit = (m == 2) ? 80.0 : 120.0;
        out = constrain(out, -descent_limit, descent_limit);
    }
    
    // **Ma sát tĩnh (Cho M2)**
	// Áp dụng trước khi Scaling gần đích, nhưng sau khi tổng hợp PID+FF+Grav
	if (m == 2 && fabs(v_target) > 0.1 && fabs(v_target) < 1.5) { // Chỉ áp dụng khi motor bắt đầu hoặc đang chạy chậm
		out += copysign(22.0, v_target);
	}
	if (m == 3 && fabs(v_target) > 0.1 && fabs(v_target) < 1.2) {
    out += copysign(18.0, v_target);
}   
    // **Giảm phản ứng M3 khi M2 xoay nhanh**
	if (m == 3 && fabs(current_velocity2) > 1.5 && fabs(posErr) < 30) {
		out *= 0.5; // Giảm 50% tổng lực đẩy
	}
    
    // **Scaling gần đích (M2, M3)**
    if (m == 2 && abs(posErr) < 5) {
        float scale = constrain(abs(posErr) / 10.0, 0.4, 1.0);
        out *= scale;
    }
    if (m == 3 && abs(posErr) < 10) {
        float scale = constrain(abs(posErr) / 10.0, 0.3, 1.0);
        out *= scale;
    }
	// LPF cho PWM
	float filt;
    // Sử dụng alpha đã định nghĩa sẵn
	if (m == 3)
		filt = 0.65 * prev_out[m] + 0.35 * out;
	else
		filt = alpha * out + (1.0 - alpha) * prev_out[m];

	prev_out[m] = filt;
	filt = constrain(filt, -pwm_limit, pwm_limit);

	applyPWM(m, filt);
}

/******************************************
 apply PWM to motor m (1..3)
 ******************************************/
void applyPWM(int m, float pwm) {
	int ipwm = abs((int)pwm);
	if (ipwm > 255) ipwm = 255;

	if (pwm > 0) {
		if (m==1) fwd1(ipwm);
		else if (m==2) fwd2(ipwm);
		else fwd3(ipwm);
	}
	else if (pwm < 0) {
		if (m==1) rev1(ipwm);
		else if (m==2) rev2(ipwm);
		else rev3(ipwm);
	}
	else {
		if (m==1) stop1();
		else if (m==2) stop2();
		else stop3();
	}
}

/******************************************
 reset integrators and small state
 ******************************************/
void resetPIDState() {
	update_encoder_readings();

	// ===== Reset encoder history =====
	prev_enc1 = enc1;
	prev_enc2 = enc2;
	prev_enc3 = enc3;

	current_velocity1 = 0;
	current_velocity2 = 0;
	current_velocity3 = 0;

	vel_lpf[1] = 0;
	vel_lpf[2] = 0;
	vel_lpf[3] = 0;

	for (int i = 1; i <= 3; i++) {
		i_pos[i] = 0.0;
		p_pos[i] = 0.0;

		i_vel[i] = 0.0;
		p_vel[i] = 0.0;

		prev_out[i] = 0.0;

		long enc = (i == 1) ? enc1 : (i == 2 ? enc2 : enc3);

		target_position[i]   = (float)enc;
		final_target_pulse[i]= enc;

		s_curve_target_vel[i]= 0.0;
		s_curve_target_pos[i]= target_position[i];

		traj_active[i] = false;
		jobDoneArr[i]  = true;
		nStable[i]     = 0;
	}

	has_moved = false;
}


/******************************************
 brake / stop helpers
 ******************************************/
void brakeAll() {
	brake1(); brake2(); brake3();
}
void stopAll() {
	stop1(); stop2(); stop3();
}

/******************************************
 Homing sequence (simple blocking)
 ******************************************/
void startHomingSequence() {
	start_homing[1] = 0; start_homing[2] = 0; start_homing[3] = 0;
	// M1
	while (start_homing[1] == 0) {
		fwd1(40);
		if (digitalRead(CTHT_1) == HIGH) {
			delay(5);
			if (digitalRead(CTHT_1) == HIGH) {
				start_homing[1] = 1;
				enc1_lib.write(0); // Reset encoder bằng thư viện
				brake1();
			}
		}
	}
	// M2
	while (start_homing[2] == 0) {
		fwd2(45);
		if (digitalRead(CTHT_2) == HIGH) {
			delay(10);
			if (digitalRead(CTHT_2) == HIGH) {
				start_homing[2] = 1;
				enc2_lib.write(0);
				brake2();
			}
		}
	}
	// M3
	while (start_homing[3] == 0) {
		fwd3(120);
		if (digitalRead(CTHT_3) == HIGH) {
			delay(10);
			if (digitalRead(CTHT_3) == HIGH) {
				start_homing[3] = 1;
				enc3_lib.write(0);
				brake3();
			}
		}
	}
	// After homing, lock pid state and targets
	update_encoder_readings();
	resetPIDState();
}

/******************************************
 setup()
 ******************************************/
void setup() {
	Serial.begin(115200);

	// pins motor 1
	pinMode(M1_IN1, OUTPUT);
	pinMode(M1_IN2, OUTPUT);
	pinMode(M1_ENA, OUTPUT);

	// motor 2
	pinMode(M2_IN1, OUTPUT);
	pinMode(M2_IN2, OUTPUT);
	pinMode(M2_ENA, OUTPUT);

	// motor 3
	pinMode(M3_IN1, OUTPUT);
	pinMode(M3_IN2, OUTPUT);
	pinMode(M3_ENA, OUTPUT);

	// sensors / solenoid
	pinMode(ENA, OUTPUT);
	pinMode(CTHT_1, INPUT_PULLUP);
	pinMode(CTHT_2, INPUT_PULLUP);
	pinMode(CTHT_3, INPUT_PULLUP);
	digitalWrite(ENA, LOW);

	// initial states
	resetPIDState();
	brakeAll();
	
	lastVel = millis();
	lastPos = millis();

	Serial.println("Controller ready.");
}

/******************************************
 loop()
 ******************************************/
void loop() {
	// parse serial incoming line (non-blocking)
	if (stringComplete) {
		inputString.trim();
		if (inputString.length() > 0) {
			if (inputString.startsWith("SET")) {
				inputString.replace("SET", "");
				inputString.trim();
				int firstSpace = inputString.indexOf(' ');
				int secondSpace = inputString.indexOf(' ', firstSpace + 1);
				if (firstSpace == -1 || secondSpace == -1) {
					Serial.println("SET format error: SET a b c (degrees)");
				} else {
					float a = inputString.substring(0, firstSpace).toFloat();
					float b = inputString.substring(firstSpace + 1, secondSpace).toFloat();
					float c = inputString.substring(secondSpace + 1).toFloat();
					
					update_encoder_readings(); 
					resetPIDState();
					pulse[1] = (long)(a * pulsesPerDegree1);
					pulse[2] = (long)(b * pulsesPerDegree2);
					pulse[3] = (long)(c * pulsesPerDegree3);
					
					final_target_pulse[1] = pulse[1];
					final_target_pulse[2] = pulse[2];
					final_target_pulse[3] = pulse[3];
					
					target_position[1] = (float)final_target_pulse[1];	
					target_position[2] = (float)final_target_pulse[2];
					target_position[3] = (float)final_target_pulse[3];

					start_position[1] = enc1;
					start_position[2] = enc2;
					start_position[3] = enc3;

					// Bắt đầu lập quỹ đạo cho tất cả motor
					plan_quintic_trajectory(1);
					plan_quintic_trajectory(2);
					plan_quintic_trajectory(3);
					//resetPIDState();
					motor_enabled = true;
					is_all_done=true;
					has_moved = false;

					
					for (int i=1;i<=3;i++) {
						i_pos[i] = 0.0;
						i_vel[i] = 0.0;
						nStable[i] = 0;
					}
					Serial.print("SET received (deg): ");
					Serial.print(a); Serial.print(", ");
					Serial.print(b); Serial.print(", ");
					Serial.println(c);
					Serial.print("SET received (pulse): ");
					Serial.print(final_target_pulse[1]); Serial.print(", ");
					Serial.print(final_target_pulse[2]); Serial.print(", ");
					Serial.println(final_target_pulse[3]);
				}
			}
			else if (inputString.equalsIgnoreCase("HOME")) {
				motor_enabled = true;
				Serial.println("Starting HOME...");
				startHomingSequence();
				Serial.println("HOME DONE");	
			}
			else if (inputString.equalsIgnoreCase("HUT")) {
				digitalWrite(ENA, HIGH);
				Serial.println("HUT DONE");
			}
			else if (inputString.equalsIgnoreCase("THA")) {
				digitalWrite(ENA, LOW);	
				Serial.println("THA DONE");
			}
			// clear input
			inputString = "";
		}
		stringComplete = false;
	}

	unsigned long now = millis();

	if (!motor_enabled) {
		brakeAll();
	} else {
		update_encoder_readings();

		// 1) inner loop - velocity PID @ 200Hz
		if (now - lastVel >= VEL_INTERVAL_MS) {
			calculate_current_velocity();	

			velocityPID(1);	
			velocityPID(2);
			velocityPID(3);
			
			lastVel = now;
		}

		// 2) outer loop - position PID @ 50Hz
		if (now - lastPos >= POS_INTERVAL_MS) {
			// Cập nhật quỹ đạo cho tất cả motor
			sample_trajectory(1);
			sample_trajectory(2);
			sample_trajectory(3);

			positionPID(1);
			positionPID(2);
			positionPID(3);

			lastPos = now;
		}
	
	// Kiểm tra đã di chuyển chưa
	if (!has_moved) {
		if (abs(enc1 - start_position[1]) > 5 ||
			abs(enc2 - start_position[2]) > 5 ||
			abs(enc3 - start_position[3]) > 5) {
			has_moved = true;
		}
	}

	// ================= DONE CHECK =================
	if (is_all_done) {

		bool traj_done =
			!traj_active[1] &&
			!traj_active[2] &&
			!traj_active[3];

		// -------- Joint 1 --------
		long e1 = final_target_pulse[1] - enc1;
		bool pos_ok1 = abs(e1) <= allowError1;
		bool vel_ok1 = fabs(current_velocity1) < 0.2;
		if (pos_ok1 && vel_ok1) nStable[1]++;
		else nStable[1] = 0;

		// -------- Joint 2 --------
		long e2 = final_target_pulse[2] - enc2;
		bool pos_ok2 = abs(e2) <= 7;	
		bool vel_ok2 = fabs(current_velocity2) < 0.2;
		if (pos_ok2 && vel_ok2) nStable[2]++;
		else nStable[2] = 0;

		// -------- Joint 3 --------
		long e3 = final_target_pulse[3] - enc3;
		bool pos_ok3 = (abs(e3) <= 6);
		bool vel_ok3 = fabs(current_velocity3) < 0.1;
		if (pos_ok3 && vel_ok3) nStable[3]++;
		else nStable[3] = 0;

		if (traj_done &&
			nStable[1] >= STABLE_COUNT &&
			nStable[2] >= STABLE_COUNT &&
			nStable[3] >= STABLE_COUNT &&
			has_moved) {

			Serial.println("ALL DONE");
			Serial.println("DONE");
			is_all_done = false;
			//resetPIDState(); // Reset PID khi hoàn tất để chuẩn bị cho lệnh tiếp theo
		}
	}

	} // end motor_enabled branch
	//update_encoder_readings();
  //printlog();
}

/******************************************
 Debug Log
 ******************************************/
void printlog()
{
	static unsigned long tlog = 0;
	if (millis() - tlog > 200) {
		Serial.print("Taget: ");
		Serial.print(final_target_pulse[1]); Serial.print(", ");
		Serial.print(final_target_pulse[2]); Serial.print(", ");
		Serial.println(final_target_pulse[3]);
		Serial.print("| Enc: ");
		Serial.print(enc1); Serial.print(", ");
		Serial.print(enc2); Serial.print(", ");
		Serial.println(enc3);
		tlog = millis();
	}
}

/******************************************
 serialEvent() - collect line
 ******************************************/
void serialEvent() {
	while (Serial.available()) {
		char inChar = (char)Serial.read();
		if (inChar == '\n' || inChar == '\r') {
			stringComplete = true;
		} else {
			inputString += inChar;
		}
	}
}

// Hàm hỗ trợ tính góc khớp theo radian (cho Grav FF)
float joint_angle_rad(int m) {
	if (m == 2) return enc2 / pulsesPerDegree2 * DEG_TO_RAD;
	if (m == 3) return enc3 / pulsesPerDegree3 * DEG_TO_RAD;
	return 0;
}