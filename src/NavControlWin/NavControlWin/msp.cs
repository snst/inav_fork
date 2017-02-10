using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NavControlWin
{
    class MSP
    {
        public enum msp
        {
                  MSP_API_VERSION = 1
                , MSP_IDENT = 100
                , MSP_STATUS = 101
                , MSP_RAW_IMU = 102
                , MSP_STSC_SONAR = 130
                , MSP_STSC_BARO = 131
//                , MSP_STSC_POS_EST = 132
                , MSP_STSC_SETFLAG = 133
                , MSP_STSC_SETSONAR = 134
                , MSP_SONAR_ALTITUDE = 58
        };
        
        public short[] adcACC = new short[3];
        public short[] adcGyro = new short[3];
        public short[] adcMag = new short[3];
        /*
        public float estimatedActualPosZ = 0;
        public float estmatedActualVelZ = 0;
        public int rangefinderLatestAlt = 0;
        public float pe_SonarAlt = 0;
        public float pe_SonarVel = 0;
        public float pe_BaroAlt = 0;
        public float pe_BaroAltFiltered = 0;
        public float[] pe_imuA_NEU = new float[3];
        public float[] pe_imuV_NEU = new float[3];
        public float[] pe_imuP_NEU = new float[3];
        public float st_alt = 0;
        public float st_alt2 = 0;
        public bool hasData = false;
        public float st_a = 0;
        public float st_v = 0;
        public float st_z = 0;

        */
        public float accZ = 0;
        public float peVelZ = 0;
        public float pePosZ = 0;
        public float sonarAlt = 0;
        public float sonarAltMean = 0;
        public float sonarVel = 0;
        public float baroAlt = 0;
        public float baroAltMean = 0;
        public float baroVel = 0;
        public float baroResidual = 0;
        public float sonarResidual = 0;
        public float baroSonarResidual = 0;
        public float baroOffset = 0;

        /*
        public float peBaroAlt = 0;
        public float baroAltOffset = 0;
        public float accelBias = 0;
        */
        public short stscFlags = 0;
        public uint readFlag = 0;
        /*      public float baroResidual = 0;
              public float sonarBaroResidual = 0;
              public float baroTemp = 0;
      */
        public float baroRawTemp = 0;
        public float baroRawPressure = 0;

        public byte[] makePaket(msp cmd, byte n)
        {
            byte[] b = new byte[6 + n];
            b[0] = (byte)'$';
            b[1] = (byte)'M';
            b[2] = (byte)'<';
            b[3] = n;
            b[4] = (byte)cmd;
            return b;
        }

        public void setInt(byte[] target, int pos, int value)
        {
            byte[] b = BitConverter.GetBytes(value);
            b.CopyTo(target, pos + 5);
        }

        public void setInt16(byte[] target, int pos, short value)
        {
            byte[] b = BitConverter.GetBytes(value);
            b.CopyTo(target, pos + 5);
        }

        public void makeCRC(byte[] b)
        {
            byte sum = 0;
            for(int i=3;i<b.Length-1; i++)
            {
                sum ^= b[i];
            }
            b[b.Length - 1] = sum;
        }

        public byte[] cmdApiVersion()
        {
            byte[] b = makePaket(msp.MSP_API_VERSION, 0);
            makeCRC(b);
            return b;
        }

        public byte[] cmdRequest(msp cmd)
        {
            byte[] b = makePaket(cmd, 0);
            makeCRC(b);
            return b;
        }

        /*
        public byte[] cmdWriteDebugVal(int index, float val)
        {
            byte[] b = makePaket(msp.MSP_STSC_WRITE_DEBUG_VAL, 8);
            setInt(b, 0, index);
            setInt(b, 4, (int)(val * 1000.0f));
            makeCRC(b);
            return b;
        }*/

        public byte[] cmdWriteFlag()
        {
            byte[] b = makePaket(msp.MSP_STSC_SETFLAG, 2);
            setInt16(b, 0, stscFlags);
            makeCRC(b);
            return b;
        }

//        public byte[] cmdSetSonar(decimal baroVal, decimal sonarVal, decimal sonarBaroVal, decimal baroSmooth, decimal baroAccCorr, decimal baroSonarCorr)
        public byte[] cmdSetSonar(decimal mode, decimal baroVelScale, decimal baroVelScaleMin, decimal baro_w_z, decimal sonar_fake, decimal sonar_w_z)
        {
            byte[] b = makePaket(msp.MSP_STSC_SETSONAR, 12);
            setInt16(b, 0, (short)(mode));
            setInt16(b, 2, (short)(baroVelScale * 100));
            setInt16(b, 4, (short)(baroVelScaleMin * 100));
            setInt16(b, 6, (short)(baro_w_z * 100));
            setInt16(b, 8, (short)(sonar_fake));
            setInt16(b, 10, (short)(sonar_w_z * 100));

            /*
            byte[] b = makePaket(msp.MSP_STSC_SETSONAR, 12);
            setInt16(b, 0, (short)(baroVal * 100));
            setInt16(b, 2, (short)(sonarVal * 100));
            setInt16(b, 4, (short)(sonarBaroVal * 100));
            setInt16(b, 6, (short)(baroSmooth * 100));
            setInt16(b, 8, (short)(baroAccCorr * 100));
            setInt16(b, 10, (short)(baroSonarCorr * 100));
*/
            makeCRC(b);
            return b;
        }

        short read16(byte[] buffer, ref int pos)
        {
            short ret = BitConverter.ToInt16(buffer, pos);
            pos += 2;
            return ret;
        }

        int readInt32(byte[] buffer, ref int pos)
        {
            int ret = BitConverter.ToInt32(buffer, pos);
            pos += 4;
            return ret;
        }

        float readFloat(byte[] buffer, ref int pos)
        {
            float ret = BitConverter.ToSingle(buffer, pos);
            pos += 4;
            return ret;
        }


        public bool process(byte[] buffer, int size)
        {
            if(buffer[0]!=(byte)'$' || buffer[1]!=(byte)'M')
                return false;

            byte sum = 0;
            for(int i=3; i<size-1;i++)
            {
                sum ^= buffer[i];
            }
            if (sum != buffer[size - 1])
                return false;

            int pos = 5;

            switch((msp)buffer[4])
            {
                case msp.MSP_API_VERSION:
                    break;

                case msp.MSP_SONAR_ALTITUDE:
                    sonarAlt = readInt32(buffer, ref pos);
                    break;

                case msp.MSP_RAW_IMU:
                    adcACC[0] = read16(buffer, ref pos);
                    adcACC[1] = read16(buffer, ref pos);
                    adcACC[2] = read16(buffer, ref pos);
                    adcGyro[0] = read16(buffer, ref pos);
                    adcGyro[1] = read16(buffer, ref pos);
                    adcGyro[2] = read16(buffer, ref pos);
                    adcMag[0] = read16(buffer, ref pos);
                    adcMag[1] = read16(buffer, ref pos);
                    adcMag[2] = read16(buffer, ref pos);
                    break;

                case msp.MSP_STSC_SONAR:
                    readFlag = (uint)read16(buffer, ref pos);
                    accZ = ((float)read16(buffer, ref pos)) / 100.0f;
                    pePosZ = read16(buffer, ref pos);
                    peVelZ = read16(buffer, ref pos);
                    sonarAlt = read16(buffer, ref pos);
                    sonarAltMean = (uint)read16(buffer, ref pos);
                    baroAlt = read16(buffer, ref pos);
                    baroAltMean = read16(buffer, ref pos);
                    baroResidual = read16(buffer, ref pos);
                    sonarResidual = read16(buffer, ref pos);
                    baroSonarResidual = read16(buffer, ref pos);
                    baroOffset = read16(buffer, ref pos);

                    /*
                                        rawBaroAlt = read16(buffer, ref pos);
                                        rawBaroAltMean = read16(buffer, ref pos);
                                        peSonarAlt = read16(buffer, ref pos);
                                        peSonarVel = read16(buffer, ref pos);
                                        peVelZ = read16(buffer, ref pos);
                                        pePosZ = read16(buffer, ref pos);
                                        peBaroAlt = read16(buffer, ref pos);
                                        baroAltOffset = read16(buffer, ref pos);
                                        accelBias = ((float)readInt32(buffer, ref pos))/100.0f;
                                        baroResidual = read16(buffer, ref pos);
                                        sonarBaroResidual = read16(buffer, ref pos);
                                        baroTemp = read16(buffer, ref pos);
                                        sonarAltMean = (uint)read16(buffer, ref pos);
                                        readFlag = (uint)read16(buffer, ref pos);
                    */
                    break;

                case msp.MSP_STSC_BARO:
                    baroRawPressure = ((float)readInt32(buffer, ref pos));
                    baroRawTemp = ((float)readInt32(buffer, ref pos));
                    break;

        //case msp.MSP_STSC_DEBUG_IMU:
        //    pe_imuA_NEU[0] = readInt32(buffer, ref pos);
        //    pe_imuA_NEU[1] = readInt32(buffer, ref pos);
        //    pe_imuA_NEU[2] = readInt32(buffer, ref pos);
        //    pe_imuV_NEU[0] = readInt32(buffer, ref pos);
        //    pe_imuV_NEU[1] = readInt32(buffer, ref pos);
        //    pe_imuV_NEU[2] = readInt32(buffer, ref pos);
        //    pe_imuP_NEU[0] = readInt32(buffer, ref pos);
        //    pe_imuP_NEU[1] = readInt32(buffer, ref pos);
        //    pe_imuP_NEU[2] = readInt32(buffer, ref pos);
        //    break;

                    //case msp.MSP_STSC_ALTITUDE:
                    //    estimatedActualPosZ = readInt32(buffer, ref pos);
                    //    estmatedActualVelZ = readInt32(buffer, ref pos);
                    //    rangefinderLatestAlt = readInt32(buffer, ref pos);
                    //    pe_SonarAlt = readInt32(buffer, ref pos);
                    //    pe_SonarVel = readInt32(buffer, ref pos);
                    //    pe_BaroAlt = readInt32(buffer, ref pos);
                    //    pe_BaroAltFiltered = readInt32(buffer, ref pos);
                    //    pe_imuA_NEU[0] = readInt32(buffer, ref pos);
                    //    pe_imuA_NEU[1] = readInt32(buffer, ref pos);
                    //    pe_imuA_NEU[2] = readInt32(buffer, ref pos);
                    //    st_alt = readInt32(buffer, ref pos);
                    //    st_alt2 = readInt32(buffer, ref pos);
                    //    break;

                    //case msp.MSP_STSC_ALTITUDE2:
                    //    pe_BaroAlt = readInt32(buffer, ref pos);
                    //    pe_imuA_NEU[2] = readInt32(buffer, ref pos);
                    //    //                    rangefinderLatestAlt = readInt32(buffer, ref pos);
                    //    st_a  = readInt32(buffer, ref pos);
                    //    st_v = readInt32(buffer, ref pos);
                    //    st_z = readInt32(buffer, ref pos);
                    //    break;

            }

    //hasData = true;

            return true;
        }

    }
}
