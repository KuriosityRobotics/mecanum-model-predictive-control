addpath 'DynamicSolver'
addpath '../../Downloads/forces_pro_client'
addpath '../../Downloads/casadi-windows-matlabR2016a-v3.5.5'
disp(ForcesFindDumpedProblems())
[model, codeoptions, outputs, additionalData, problems] =  ForcesLoadSymbolicDump( 'DynamicSolver_SC015F9SE8O_20225301456404177_A.json' );


stages2qcqp(@DynamicSolver, problems, problems(1))