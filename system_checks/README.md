## System Checks

These System Checks are isolated pieces of Beaker's functionality, 
meant to ensure higher level, more abstract code is sitting on sane code.

They are not tests (not unit tests, etc..), they are meant to be run on 
Beaker's arduino and print information on Serial Out to prove out basic 
low-level functionality.

Each system check tells you exactly what to check for in the comments.

### Suggested Order of System Checks if using PWM for Motor Control:

1. Encoders
2. pwmMotor
3. servoMotor
4. wheels