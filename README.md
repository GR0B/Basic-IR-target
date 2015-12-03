(migrated from Bitbucket)

Basic IR Lasertag target. 

This target is designed to be a basic, low cost, simple and multi system IR target. 

The target is designed to respond to any signal that that uses the Sony IR protocol (Miles V1, Miles V2, Fragtag). The target 
does not decode or validate the signal more than just checking that it is long enough to contain data so it is still possible 
that it could be triggered by interference.

The target has LED, Buzzer and Vibration motor that are all optional and can be removed if desired. 
The IR receiver used is a 56khz TSOP34856 this can be substituted if another carrier frequency is required. 

The Target accepts a voltage between 3-5volts but if using the vibration motor requires a battery that can supply about 60mA 
peak so can not be run off a cr2032 with motor. Has been tested with 3xLR44, 3xAAA and 3.7v LiPo. 
Battery life when powered by 3xLR44 with light use was over 1 week. 
Standby  500Hrs [200mAh(LR44) * 0.4mAh(sleeping) = 500hrs]
Peak     3.3Hrs [200mAh(LR44) * 60mAh(non-stop)  = 3.3Hrs (would need to be hit every 0.25 seconds to use this much power)
Power Estimates (if using 200mAh 3x 1.5v LR44) 
1  hit  per minute = 142Hrs
10 hits per minute = 19Hrs
 
The code will put the ATTINY into a deep sleep after 10 seconds of no activity to reduce power usage. To Run this code on a
ATMEGA or Arduino you will need to remove or port the sleep code.   

Code size is about 1.5kb compiled so is possible to run on smaller ATTINY processors. 











Copyright (c) 2015 Bearocratic Designs (Robert Sturzbecher)

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.