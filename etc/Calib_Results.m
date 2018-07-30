% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 797.347102005416332 ; 791.754369147933517 ];

%-- Principal point:
cc = [ 491.846653790975836 ; 356.337459613377916 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.058963524719130 ; -0.156851906268213 ; 0.001307777114088 ; 0.001560428850426 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.337241671636265 ; 1.373609160684873 ];

%-- Principal point uncertainty:
cc_error = [ 2.961432073094887 ; 2.710510772307604 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.009227658503406 ; 0.027828590090207 ; 0.001242661782870 ; 0.001286528905763 ; 0.000000000000000 ];

%-- Image size:
nx = 960;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 10;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.816883e+00 ; -6.339079e-02 ; -7.385877e-03 ];
Tc_1  = [ -1.166320e+02 ; 6.494460e+01 ; 3.096426e+02 ];
omc_error_1 = [ 3.832989e-03 ; 1.075295e-03 ; 5.394879e-03 ];
Tc_error_1  = [ 1.162721e+00 ; 1.103736e+00 ; 8.646448e-01 ];

%-- Image #2:
omc_2 = [ 2.680203e+00 ; -1.788707e-01 ; -1.099674e+00 ];
Tc_2  = [ -3.886641e+01 ; 6.788083e+01 ; 4.084600e+02 ];
omc_error_2 = [ 3.676747e-03 ; 1.959543e-03 ; 5.077331e-03 ];
Tc_error_2  = [ 1.498691e+00 ; 1.389178e+00 ; 7.524030e-01 ];

%-- Image #3:
omc_3 = [ 2.521936e+00 ; -6.093381e-01 ; -9.915656e-01 ];
Tc_3  = [ -5.439891e+01 ; 9.225411e+01 ; 4.182277e+02 ];
omc_error_3 = [ 3.746584e-03 ; 2.033379e-03 ; 5.069570e-03 ];
Tc_error_3  = [ 1.536173e+00 ; 1.434969e+00 ; 8.687819e-01 ];

%-- Image #4:
omc_4 = [ 2.261140e+00 ; -1.073346e+00 ; -8.385664e-01 ];
Tc_4  = [ -2.809717e+01 ; 8.441936e+01 ; 3.934485e+02 ];
omc_error_4 = [ 3.593168e-03 ; 2.304807e-03 ; 4.948294e-03 ];
Tc_error_4  = [ 1.445248e+00 ; 1.351259e+00 ; 8.417232e-01 ];

%-- Image #5:
omc_5 = [ NaN ; NaN ; NaN ];
Tc_5  = [ NaN ; NaN ; NaN ];
omc_error_5 = [ NaN ; NaN ; NaN ];
Tc_error_5  = [ NaN ; NaN ; NaN ];

%-- Image #6:
omc_6 = [ 1.697437e+00 ; -1.692372e+00 ; -5.425349e-01 ];
Tc_6  = [ 3.849384e+01 ; 6.336176e+01 ; 3.313479e+02 ];
omc_error_6 = [ 3.041744e-03 ; 2.858415e-03 ; 4.616433e-03 ];
Tc_error_6  = [ 1.244274e+00 ; 1.139952e+00 ; 7.936462e-01 ];

%-- Image #7:
omc_7 = [ 2.674554e+00 ; -1.982755e-01 ; -1.096465e+00 ];
Tc_7  = [ -3.175432e+01 ; 7.701946e+01 ; 3.988479e+02 ];
omc_error_7 = [ 3.648009e-03 ; 1.887203e-03 ; 5.019335e-03 ];
Tc_error_7  = [ 1.466694e+00 ; 1.355682e+00 ; 7.399453e-01 ];

%-- Image #8:
omc_8 = [ 2.678649e+00 ; 2.662843e-01 ; 1.116575e+00 ];
Tc_8  = [ -1.345220e+02 ; 1.919394e+01 ; 2.645979e+02 ];
omc_error_8 = [ 4.189608e-03 ; 1.751801e-03 ; 5.364955e-03 ];
Tc_error_8  = [ 1.001054e+00 ; 9.700498e-01 ; 9.529841e-01 ];

%-- Image #9:
omc_9 = [ 2.021862e+00 ; 1.251700e-03 ; 9.813894e-02 ];
Tc_9  = [ -1.133378e+02 ; 9.576570e+01 ; 2.864940e+02 ];
omc_error_9 = [ 3.409664e-03 ; 2.031696e-03 ; 3.963686e-03 ];
Tc_error_9  = [ 1.101395e+00 ; 1.051048e+00 ; 9.007983e-01 ];

%-- Image #10:
omc_10 = [ -1.566072e+00 ; -1.474860e+00 ; -8.910361e-01 ];
Tc_10  = [ -7.813928e+01 ; -3.170304e+01 ; 1.845087e+02 ];
omc_error_10 = [ 2.543392e-03 ; 3.752532e-03 ; 3.687803e-03 ];
Tc_error_10  = [ 6.874330e-01 ; 6.464403e-01 ; 5.459849e-01 ];

