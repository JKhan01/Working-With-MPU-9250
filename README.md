# Working-With-MPU-9250
> The repository comprises of the different **filters** and approach adopted by me to obtain stable values as output.
> The development of the code has been done based upon the *mathematical understanding* of the filters.

> The library for obtaining **raw values** from MPU 9250 is available here:
>> The library has been infamously provided by [Bolder Flight Systems.][1]

> The reference paper for this project is:
>> [Data Fusion with 9DOF Inertial Measurement Unit to Determine Object's Orientation][2] by **Mr.Long Trans** of California Polytechnic State University.

> The 2 Filters I have tried here are the :
>> * Complementary Filter. (*Implementation Code is on [complementaryFilter][3] branch*)
>> * Kalman Filter. (*Implementation Code is on [kalmanFilter][4] branch*)









[1]: https://github.com/bolderflight/MPU9250
[2]: https://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?article=1422&context=eesp
[3]: https://github.com/JKhan01/Working-With-MPU-9250/tree/complementaryFilter
[4]: https://github.com/JKhan01/Working-With-MPU-9250/tree/kalmanFilter