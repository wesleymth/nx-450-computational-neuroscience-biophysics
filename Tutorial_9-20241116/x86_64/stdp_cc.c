/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__STDPSynCC
#define _nrn_initial _nrn_initial__STDPSynCC
#define nrn_cur _nrn_cur__STDPSynCC
#define _nrn_current _nrn_current__STDPSynCC
#define nrn_jacob _nrn_jacob__STDPSynCC
#define nrn_state _nrn_state__STDPSynCC
#define _net_receive _net_receive__STDPSynCC 
#define state state__STDPSynCC 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define tau _p[0]
#define tau_columnindex 0
#define e _p[1]
#define e_columnindex 1
#define gbar _p[2]
#define gbar_columnindex 2
#define tau_0 _p[3]
#define tau_0_columnindex 3
#define tau_r _p[4]
#define tau_r_columnindex 4
#define tau_y _p[5]
#define tau_y_columnindex 5
#define A_m _p[6]
#define A_m_columnindex 6
#define A_p _p[7]
#define A_p_columnindex 7
#define tetam _p[8]
#define tetam_columnindex 8
#define tetap _p[9]
#define tetap_columnindex 9
#define delay_steps _p[10]
#define delay_steps_columnindex 10
#define um1s _p[11]
#define um1s_columnindex 11
#define um2s _p[12]
#define um2s_columnindex 12
#define t_last_pre _p[13]
#define t_last_pre_columnindex 13
#define g_update _p[14]
#define g_update_columnindex 14
#define pointless_counter _p[15]
#define pointless_counter_columnindex 15
#define i _p[16]
#define i_columnindex 16
#define delay_array_u_m1 (_p + 17)
#define delay_array_u_m1_columnindex 17
#define delay_array_u_m2 (_p + 2017)
#define delay_array_u_m2_columnindex 2017
#define g _p[4017]
#define g_columnindex 4017
#define u_m1 _p[4018]
#define u_m1_columnindex 4018
#define u_m2 _p[4019]
#define u_m2_columnindex 4019
#define r _p[4020]
#define r_columnindex 4020
#define delay_array_v (_p + 4021)
#define delay_array_v_columnindex 4021
#define Dg _p[6021]
#define Dg_columnindex 6021
#define Du_m1 _p[6022]
#define Du_m1_columnindex 6022
#define Du_m2 _p[6023]
#define Du_m2_columnindex 6023
#define Dr _p[6024]
#define Dr_columnindex 6024
#define v _p[6025]
#define v_columnindex 6025
#define _g _p[6026]
#define _g_columnindex 6026
#define _tsav _p[6027]
#define _tsav_columnindex 6027
#define _nd_area  *_ppvar[0]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 0, 0
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "tau", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "tau", "ms",
 "e", "mV",
 "g", "uS",
 "i", "nA",
 0,0
};
 static double delta_t = 0.01;
 static double g0 = 0;
 static double r0 = 0;
 static double u_m20 = 0;
 static double u_m10 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, _Memb_list*, int);
static void _ode_matsol(NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[2]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"STDPSynCC",
 "tau",
 "e",
 "gbar",
 "tau_0",
 "tau_r",
 "tau_y",
 "A_m",
 "A_p",
 "tetam",
 "tetap",
 "delay_steps",
 "um1s",
 "um2s",
 "t_last_pre",
 "g_update",
 "pointless_counter",
 0,
 "i",
 "delay_array_u_m1[2000]",
 "delay_array_u_m2[2000]",
 0,
 "g",
 "u_m1",
 "u_m2",
 "r",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 6028, _prop);
 	/*initialize range parameters*/
 	tau = 0.1;
 	e = 0;
 	gbar = 0.05;
 	tau_0 = 10;
 	tau_r = 15;
 	tau_y = 114;
 	A_m = 1e-05;
 	A_p = 0.00012;
 	tetam = -64.9;
 	tetap = -35;
 	delay_steps = 50;
 	um1s = 0;
 	um2s = 0;
 	t_last_pre = -1;
 	g_update = 0;
 	pointless_counter = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 6028;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _stdp_cc_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 6028, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 STDPSynCC /Users/wesleymonteith/code/nx-450-computational-neuroscience-biophysics/Tutorial_9-20241116/stdp_cc.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset = 0; {
   double _lx , _lu_sig , _lu_m1_sig , _lu_m2_sig , _lset_loc , _lretrieve_loc ;
 if ( ( v - tetap ) > 0.0 ) {
     _lu_sig = v - tetap ;
     }
   else {
     _lu_sig = 0.0 ;
     }
   if ( ( delay_array_u_m1 [ ((int) _lretrieve_loc ) ] - tetam ) > 0.0 ) {
     _lu_m1_sig = delay_array_u_m1 [ ((int) _lretrieve_loc ) ] - tetam ;
     um1s = _lu_m1_sig ;
     }
   else {
     _lu_m1_sig = 0.0 ;
     um1s = 0.0 ;
     }
   if ( ( delay_array_u_m2 [ ((int) _lretrieve_loc ) ] - tetam ) > 0.0 ) {
     _lu_m2_sig = delay_array_u_m2 [ ((int) _lretrieve_loc ) ] - tetam ;
     um2s = _lu_m2_sig ;
     }
   else {
     _lu_m2_sig = 0.0 ;
     um2s = 0.0 ;
     }
   if ( t_last_pre  == 1.0 ) {
     _lx = 1.0 ;
     t_last_pre = 0.5 ;
     }
   else {
     _lx = 0.0 ;
     }
   Dg = - g / tau ;
   Du_m1 = ( v - u_m1 ) / tau_0 ;
   Du_m2 = ( v - u_m2 ) / tau_y ;
   Dr = ( _lx - r ) / tau_r ;
   g_update = - A_m * _lx * _lu_m1_sig + A_p * _lu_sig * r * _lu_m2_sig ;
   gbar = gbar + g_update ;
   _lset_loc = fmod ( pointless_counter , delay_steps ) ;
   _lretrieve_loc = fmod ( pointless_counter + delay_steps + 1.0 , delay_steps ) ;
   delay_array_u_m1 [ ((int) _lset_loc ) ] = u_m1 ;
   delay_array_u_m2 [ ((int) _lset_loc ) ] = u_m2 ;
   pointless_counter = pointless_counter + 1.0 ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
 double _lx , _lu_sig , _lu_m1_sig , _lu_m2_sig , _lset_loc , _lretrieve_loc ;
 if ( ( v - tetap ) > 0.0 ) {
   _lu_sig = v - tetap ;
   }
 else {
   _lu_sig = 0.0 ;
   }
 if ( ( delay_array_u_m1 [ ((int) _lretrieve_loc ) ] - tetam ) > 0.0 ) {
   _lu_m1_sig = delay_array_u_m1 [ ((int) _lretrieve_loc ) ] - tetam ;
   um1s = _lu_m1_sig ;
   }
 else {
   _lu_m1_sig = 0.0 ;
   um1s = 0.0 ;
   }
 if ( ( delay_array_u_m2 [ ((int) _lretrieve_loc ) ] - tetam ) > 0.0 ) {
   _lu_m2_sig = delay_array_u_m2 [ ((int) _lretrieve_loc ) ] - tetam ;
   um2s = _lu_m2_sig ;
   }
 else {
   _lu_m2_sig = 0.0 ;
   um2s = 0.0 ;
   }
 if ( t_last_pre  == 1.0 ) {
   _lx = 1.0 ;
   t_last_pre = 0.5 ;
   }
 else {
   _lx = 0.0 ;
   }
 Dg = Dg  / (1. - dt*( ( - 1.0 ) / tau )) ;
 Du_m1 = Du_m1  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_0 )) ;
 Du_m2 = Du_m2  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_y )) ;
 Dr = Dr  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_r )) ;
 g_update = - A_m * _lx * _lu_m1_sig + A_p * _lu_sig * r * _lu_m2_sig ;
 gbar = gbar + g_update ;
 _lset_loc = fmod ( pointless_counter , delay_steps ) ;
 _lretrieve_loc = fmod ( pointless_counter + delay_steps + 1.0 , delay_steps ) ;
 delay_array_u_m1 [ ((int) _lset_loc ) ] = u_m1 ;
 delay_array_u_m2 [ ((int) _lset_loc ) ] = u_m2 ;
 pointless_counter = pointless_counter + 1.0 ;
  return 0;
}
 /*END CVODE*/
 static int state (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) { {
   double _lx , _lu_sig , _lu_m1_sig , _lu_m2_sig , _lset_loc , _lretrieve_loc ;
 if ( ( v - tetap ) > 0.0 ) {
     _lu_sig = v - tetap ;
     }
   else {
     _lu_sig = 0.0 ;
     }
   if ( ( delay_array_u_m1 [ ((int) _lretrieve_loc ) ] - tetam ) > 0.0 ) {
     _lu_m1_sig = delay_array_u_m1 [ ((int) _lretrieve_loc ) ] - tetam ;
     um1s = _lu_m1_sig ;
     }
   else {
     _lu_m1_sig = 0.0 ;
     um1s = 0.0 ;
     }
   if ( ( delay_array_u_m2 [ ((int) _lretrieve_loc ) ] - tetam ) > 0.0 ) {
     _lu_m2_sig = delay_array_u_m2 [ ((int) _lretrieve_loc ) ] - tetam ;
     um2s = _lu_m2_sig ;
     }
   else {
     _lu_m2_sig = 0.0 ;
     um2s = 0.0 ;
     }
   if ( t_last_pre  == 1.0 ) {
     _lx = 1.0 ;
     t_last_pre = 0.5 ;
     }
   else {
     _lx = 0.0 ;
     }
    g = g + (1. - exp(dt*(( - 1.0 ) / tau)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau ) - g) ;
    u_m1 = u_m1 + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_0)))*(- ( ( ( v ) ) / tau_0 ) / ( ( ( ( - 1.0 ) ) ) / tau_0 ) - u_m1) ;
    u_m2 = u_m2 + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_y)))*(- ( ( ( v ) ) / tau_y ) / ( ( ( ( - 1.0 ) ) ) / tau_y ) - u_m2) ;
    r = r + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_r)))*(- ( ( ( _lx ) ) / tau_r ) / ( ( ( ( - 1.0 ) ) ) / tau_r ) - r) ;
   g_update = - A_m * _lx * _lu_m1_sig + A_p * _lu_sig * r * _lu_m2_sig ;
   gbar = gbar + g_update ;
   _lset_loc = fmod ( pointless_counter , delay_steps ) ;
   _lretrieve_loc = fmod ( pointless_counter + delay_steps + 1.0 , delay_steps ) ;
   delay_array_u_m1 [ ((int) _lset_loc ) ] = u_m1 ;
   delay_array_u_m2 [ ((int) _lset_loc ) ] = u_m2 ;
   pointless_counter = pointless_counter + 1.0 ;
   }
  return 0;
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{  double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _thread = (Datum*)0; _nt = (NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = g;
    double __primary = (gbar) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau ) - __primary );
    g += __primary;
  } else {
 g = gbar ;
     }
 t_last_pre = 1.0 ;
   } }
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 4; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
  int _i; double _save;{
  g = g0;
  r = r0;
  u_m2 = u_m20;
  u_m1 = u_m10;
 {
   g = 0.0 ;
   u_m1 = v ;
   u_m2 = v ;
   r = 0.0 ;
   {int  _li ;for ( _li = 0 ; _li <= 2000 ; _li ++ ) {
     delay_array_v [ _li ] = v ;
     delay_array_u_m1 [ _li ] = 0.0 ;
     delay_array_u_m2 [ _li ] = 0.0 ;
     } }
   }
 
}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   i = g * ( v - e ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   state(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = g_columnindex;  _dlist1[0] = Dg_columnindex;
 _slist1[1] = u_m1_columnindex;  _dlist1[1] = Du_m1_columnindex;
 _slist1[2] = u_m2_columnindex;  _dlist1[2] = Du_m2_columnindex;
 _slist1[3] = r_columnindex;  _dlist1[3] = Dr_columnindex;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/Users/wesleymonteith/code/nx-450-computational-neuroscience-biophysics/Tutorial_9-20241116/stdp_cc.mod";
static const char* nmodl_file_text = 
  "COMMENT\n"
  "Implementation of the STDP rule by Clopath et al., Nat. Neurosci. 13(3):344-352 2010\n"
  "\n"
  "B. Torben-Nielsen, Hebrew University\n"
  "C. Clopath, Center for Theoretical Neuroscience, Columbia U. \n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "       POINT_PROCESS STDPSynCC\n"
  "       RANGE tau, e, i : the parameters dealing with the original synapse\n"
  "       RANGE A_m, A_p, tau_y, tetam,tetap,tau_r,tau_0 : claudia's model\n"
  "       RANGE t_last_pre : 0 without presynaptic spike, 1 when a presynaptic spike occured\n"
  "       RANGE u_m1, u_m2,r,g_update,gbar\n"
  "       RANGE delay_steps,delay_array_u_m1, delay_array_u_m2, pointless_counter \n"
  "       RANGE um2s,um1s\n"
  "       NONSPECIFIC_CURRENT i\n"
  "}\n"
  "\n"
  "DEFINE MAX_ARRAY 2000 : delay_steps cannot be higher than this number\n"
  "\n"
  "UNITS {\n"
  "      (nA) = (nanoamp)\n"
  "      (mV) = (millivolt)\n"
  "      (uS) = (microsiemens)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	  : parameters for alpha synapse\n"
  "	  tau = 0.1 (ms) <1e-9,1e9>\n"
  "	  e = 0	(mV)\n"
  "	  gbar = 0.05\n"
  "\n"
  "	  : STDP model parameters\n"
  "	  tau_0 = 10 : time constant for filtering membrane potential v (called u in the original)\n"
  "	  tau_r =15 : time constant for low-pass r\n"
  "	  tau_y = 114 : time constant for post filter for potentiation\n"
  "	  A_m = 0.00001 : amplitude for depression\n"
  "	  A_p = 0.00012: amplitude for potentiation\n"
  "	  tetam = -64.9 : threshold for depression \n"
  "	  tetap = -35 : threshold for potentiation \n"
  "	  delay_steps = 50 : avoid interference from the AP, set to: AP_width / DT\n"
  "\n"
  "	  : to make some local variables accessible\n"
  "	  um1s\n"
  "	  um2s\n"
  "\n"
  "	  : internel parameters\n"
  "	  t_last_pre = -1\n"
  "	  g_update\n"
  "	  pointless_counter = 0\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	 v (mV)\n"
  "	 i (nA)\n"
  "	 delay_array_u_m1[MAX_ARRAY]\n"
  "	 delay_array_u_m2[MAX_ARRAY]\n"
  "	 delay_array_v[MAX_ARRAY]\n"
  "}\n"
  "\n"
  "STATE {\n"
  "      g (uS)\n"
  "      u_m1\n"
  "      u_m2\n"
  "      r\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	g=0\n"
  "	u_m1 = v \n"
  "	u_m2 = v \n"
  "	r = 0\n"
  "	FROM i=0 TO MAX_ARRAY {\n"
  "	     delay_array_v[i] = v\n"
  "	     delay_array_u_m1[i] = 0\n"
  "	     delay_array_u_m2[i] = 0\n"
  "	}\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	   SOLVE state METHOD cnexp\n"
  "	   i = g*(v - e)\n"
  "}\n"
  "\n"
  "DERIVATIVE state { \n"
  "	   LOCAL x,u_sig,u_m1_sig,u_m2_sig,set_loc,retrieve_loc \n"
  "\n"
  "	  : compute the sigmas\n"
  "	  if( (v - tetap) >  0) {\n"
  "	      u_sig = v - tetap \n"
  "	      }\n"
  "	  else { u_sig = 0 }\n"
  "\n"
  "	  if( (delay_array_u_m1[retrieve_loc] - tetam) > 0) {\n"
  "	      u_m1_sig = delay_array_u_m1[retrieve_loc] - tetam :\n"
  "	      um1s = u_m1_sig\n"
  "	  }\n"
  "	  else { \n"
  "	       u_m1_sig = 0 \n"
  "	       um1s = 0\n"
  "	  }\n"
  "	  if( (delay_array_u_m2[retrieve_loc] - tetam) > 0 ) {\n"
  "	      u_m2_sig = delay_array_u_m2[retrieve_loc] - tetam :\n"
  "	      um2s = u_m2_sig\n"
  "	  }\n"
  "	  else { \n"
  "	       u_m2_sig = 0 \n"
  "	       um2s = 0\n"
  "	  }\n"
  "\n"
  "	  if(t_last_pre == 1) {\n"
  "	  	x = 1\n"
  "	    	t_last_pre = 0.5\n"
  "	    	:printf(\"x is one\")\n"
  "	  } else {\n"
  "	    	x = 0\n"
  "	  }\n"
  "\n"
  "	  g' = -g/tau\n"
  "	  u_m1' = (v-u_m1)/tau_0 \n"
  "	  u_m2' = (v-u_m2)/tau_y\n"
  "	  r' = (x-r)/ tau_r\n"
  "	  g_update = - A_m*x*u_m1_sig + A_p*u_sig*r*u_m2_sig \n"
  "	  gbar = gbar + g_update\n"
  "\n"
  "	  : avoid interference of the AP\n"
  "	  set_loc = fmod(pointless_counter,delay_steps)\n"
  "	  retrieve_loc = fmod(pointless_counter+delay_steps+1,delay_steps)\n"
  "	  delay_array_u_m1[set_loc] = u_m1\n"
  "	  delay_array_u_m2[set_loc] = u_m2\n"
  "	  pointless_counter = pointless_counter + 1 \n"
  "}\n"
  "\n"
  "COMMENT\n"
  "modified from original AlphaSynapse: when a presynaptic event is received the conductance is always set to the max conductance. Even with multiple successive events, the conductance is not summend and will not go higher than gbar.\n"
  "ENDCOMMENT\n"
  "NET_RECEIVE(weight (uS)) {\n"
  "		   g = gbar : set the max conductance of the synapse\n"
  "		   t_last_pre = 1\n"
  "}\n"
  ;
#endif
