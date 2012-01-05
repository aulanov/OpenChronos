#include "project.h"

#include "altitude_math.h"

static s16 pRef = 101325/4; // Reference pressure at sea level in 4Pa units
static s16 pLast = 101325/4; // Last measured pressure in 4Pa units
static s16 hLast = 0; // Last altitude estimate in normalized units b/H0/2^15

// *************************************************************************************************
// @fn          conv_altitude_to_fraction
// @brief       Relative pressure deviation from reference pressure for given altitude estimate.
// @param       s16 hh Altitude estimate (in normalised units).
// @return      Calculated relative pressure deviation for this altitude
// *************************************************************************************************
s16 conv_altitude_to_fraction(s16 hh)
{
	/*
	The fixed part of the function of altitude can be broken into tabulated ranges
	and/or interpolated according to a Taylor series expansion
		 (1 - f) = (1 - h/H0)^b
		         = 1 - h*b/H0 + h^2*b*(b-1)/2/H0^2 - h^3*b8(b-1)*(b-2)/6/H0^3 + ...
	At low altitudes h/H0 << 1, so this series tends to converge rapidly and is
	well-suited for fixed point implementation. With one or two additional terms
	the series converges accurately over the range of interest so there is no need
	for table interpolation. For the proposed fixed point implementation we rewrite
	this expression a bit into
		hh = b*h/H0
		(1 - f) = (1 - h/H0)^b
		        = 1 - hh*(1 - hh*(b-1)/2/b*(1 - hh*(b-2)/3/b*(...
	We stick to integer multiply and shift operations. Signed s16 values can contain
	values +/-2^15 and unsigned u16 values 0..2^16. In C multiplication amounts to
	expanding to s32, integer multiply and scaling back by a proper shift operation.

	Given the above equations the natural unit of hh as the first order correction is
	H0/b = 8434.48m. If we accept this as a maximum +/- range we can store s16 hh in
	units of (H0/b)/2^15 = 0,26m which keeps the resolution at less than a foot.
	 */
	s16 f, hf;
	// f  = hh*(b - 4)/5/b, correction relevant above 3.5km:
	// (Could be omitted, but it is relatively little work.)
	f = mult_scale16(hh, 3132);
	// f  = hh*(b - 3)/4/b*(1 - f), correction relevant above 1.3km:
	hf = mult_scale16(hh, 7032);
	f = hf - mult_scale15(hf,f);
	// f = hh*(b - 2)/3/b*(1 - f), correction relevant above 300m:
	hf = mult_scale16(hh, 13533);
	f = hf - mult_scale15(hf,f);
	// f = hh*(b - 1)/2/b*(1 - f), correction relevant above 30m:
	hf = mult_scale16(hh, 26533);
	f = hf - mult_scale15(hf,f);
	// f = hh*(1 - f), the linear part:
	f = hh - mult_scale15(hh,f);
	return f;
}

// *************************************************************************************************
// @fn          update_pressure_table
// @brief       Calculate pressure table for reference altitude.
//				Implemented straight from VTI reference code.
// @param       s16		href	Reference height
//				u32		p_meas	Pressure (Pa)
// @return     	none
// *************************************************************************************************
void update_pressure_table(s16 href, u32 p_meas)
{
	// Note: a user-provided sea-level reference pressure in mbar as used by pilots
	// would be straightforward: href = 0; p_meas = (s32)mbar*100;
	// The altitude reading will be iteratively updated.

	// Convert to 4Pa units:
	pLast = (s16)((p_meas+2) >> 2);
	// Convert reference altitude to normalized units:
	if (sys.flag.use_metric_units) { // user_altitude in m
		hLast = 4*href - mult_scale16(href, 7536);
	} else { // user_altitude in ft
		hLast = href + mult_scale16(href,12068);
	}
	s32 f = (s32)0x8000 - conv_altitude_to_fraction(hLast);
	// pRef = p_meas*2^15/f:
	pRef = ((((s32)pLast << 16) + f) >> 1) / f;
	// The long division is acceptable because it happens rarely.
	// The term + f) is for proper rounding.
	// The <<16 and >>1 operations correct for the 15bit scale of f.
}

// *************************************************************************************************
// @fn          conv_pressure_to_altitude
// @brief       Calculates altitude from current pressure, and
//				stored reference pressure at sea level and previous altitude estimate.
//				Temperature info is ignored.
// @param       u32		p_meas	Pressure (Pa)
// @return      Estimated altitude in user-selected unit (m or ft)
//              (internally filtered, slightly sluggish).
// *************************************************************************************************
s16 conv_pa_to_altitude(u32 p_meas)
{
	/*
	Assumption: fixed, linear T(h)
	T = T0 - dTdh*h
	with
	T0 = 288.15K (15C)
	dTdh = 6.5mK/m

	Basic differential equation:
		dh = -(R/G)*T(H)*dp/p
	Solution:
		H = H0*(1 - (p/pRef)^a)
	with
		H0 = T0/dTdh = 44330.77m
		pRef = adjustable reference pressure at sea level (h=0).
		a = dTdH*R/G = 0.190263
		R = 287.052m^2/s^2/K
		G = 9.80665 (at medium latitude)

	We assume T0 and the temperature profile to be fixed; the temperature reading
	of the watch is not very useful since it is strongly influenced by body heat,
	clothing, shelter, etc.

	Straight evaluation of h(p) requires an unattractive long division p/pRef
	with pRef the adjustable reference pressure, and the Taylor expansion does
	not converge very quickly.

	Evaluation of p(h) requires a more attractive multiplication by the
	user-adjustable reference pressure pRef:
		f =(1 - h/H0)^b
		p = pRef*f
	with
		b = 1/a = G/(dTdH*R) = 5.255896
	In a very crude linear iteration the h value can be updated by
		delta_h = -delta_p / dpdh
	The slope dpdh varies by about a factor two over the range of interest,
	but we can pick a fixed value on the safe side and accept that the updates
	are a bit more damped at higher altitudes.

	The sensor provides 19bit absolute pressure in units of 0.25Pa, but that is more
	resolution than we can easily handle in the multiplications. We store measured
	pressure p, reference pressure pRef and calculated pressure as u16 in units of 4Pa.

	In the units chosen for p (4Pa) and for hLast (see function conv_altitude_to_fraction),
	the slope dpdh is about -0.75 at sea level down to -0.375 at high altitudes. To avoid
	overshoot and instabilities we assume a bigger value and accept a minor amount of
	extra filtering delay at higher altitudes. The factor 1/0.75 is approximated by 1.
	*/
	// Scale to 4Pa units:
	s16 p = (s16)((p_meas+2) >> 2);
	// Store current pressure for next predictor:
	pLast = p;
	// Calculate pressure ratio based on guessed altitude (serious DSP work):
	s16 f = conv_altitude_to_fraction(hLast);
	// Calculate pressure expected for guessed height
	u16 pCalculated = pRef - mult_scale15(pRef,f);
	// This calculation is correct within about 7Pa.
	// Reverse the solution with a linearly improved guess:
	hLast -= mult_scale15((p - pCalculated)<<1, 21845);

	if (sys.flag.use_metric_units) {
		// Altitude in meters (correct within about 0.7m):
		return mult_scale16(hLast, 16869);
	} else {
		// Altitude in feet (correct within 1.5ft):
		return mult_scale15(hLast, 27672);
	}
}
