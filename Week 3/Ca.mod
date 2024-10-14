NEURON	{
	SUFFIX Ca
	USEION ca READ eca WRITE ica
	RANGE gCabar, gCa, ica 
}


UNITS	{
	(S) = (siemens)
	(mV) = (millivolt)
	(mA) = (milliamp)
}

PARAMETER	{
	gCabar = 0.00001 (S/cm2) 
    scale_mTau = 3
    vhalf_mTau = -75
    vhalf_mInf = -27
    scale_hTau = 100
    vhalf_hTau = -27
    vhalf_hInf = -27
}

STATE	{ 
	m
	h
}

ASSIGNED	{
	v	(mV)
	eca	(mV)
	ica	(mA/cm2)
	gCa	(S/cm2)
	mInf
	mTau
	hInf
	hTau
}



BREAKPOINT	{
	SOLVE states METHOD cnexp
	gCa = gCabar*m*m*h
	ica = gCa*(v-eca)
}

DERIVATIVE states	{
	rates()
	m' = (mInf-m)/mTau
	h' = (hInf-h)/hTau
}

INITIAL{
	rates()
	m = mInf
	h = hInf
}

PROCEDURE rates(){
	UNITSOFF
        mInf = 1 / (1 + exp(vhalf_mInf - v))
        mTau = scale_mTau / (1 + exp(vhalf_mTau - v))
        hInf = 1 - (1 / (1 + exp(vhalf_hInf-v)))
        hTau = scale_hTau * (1 / (1 + exp(vhalf_hTau-v)))
	UNITSON
}