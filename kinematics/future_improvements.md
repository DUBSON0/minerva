# This document is for development process: contains future steps/plans to make progress

Make initial tunning easier - instead of requiring angle-zero, allow to insert angles, a_home, p_home - when robot arm is straightened, it is easier to measure these things

a_home should be normalized - maybe it would be more convenient to normalize it in code

Add constrains to angles so that the stay within the range of servo motors - this is also connected to above: do you tune home position with arbitrary angles and then insert a little more complicated bounds for angles, or do you just bound the angles by [-pi,pi] - whatever the servo limits are - and then write a separate tunning scheme, where you input angles, and arm positions - then it calculates appropriate tuning for when all angles are zero. Separate tuning scheme might make more sense - we can also use data analysis to have different methods of tuning (e.g. multiple positions for different angles), and higher accuracy.

Allow for many solutions. This is more of a vibe improvement in order to allow for better optimization (e.g. small continuous/local optimizations). The problem is clearer when singularity occurs - this is a separate problem and needs discrete/combinatorial approach to break the symmetries (e.g. when arm is at a singularity/extended, does it approach the target point from left of right?)