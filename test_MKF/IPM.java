/*
 * Copyright (C) 2019 Pablo Bernal-Polo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


// implemented Information Packets:
// information packet ID - name
// 0 - IPM_MPU6050
// 1 - IPM_MPU6050_HMC5883L
// 2 - IPM_BMP085
// 3 - IPM_AdafruitIMU9dof
// 4 - IPM_t_MPU6050_HMC5883L
// 5 - IPM_t_BMP085


public class IPM {
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // PARAMETERS
  ///////////////////////////////////////////////////////////////////////////////////////
  protected static final int NON_MEASUREMENT_BYTES = 2;  // measurement ID + sensor ID
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // VARIABLES
  ///////////////////////////////////////////////////////////////////////////////////////
  protected byte[] b;
  
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // PUBLIC METHODS
  ///////////////////////////////////////////////////////////////////////////////////////
  
  public void set_bytes( byte[] bytePointer ) {
    this.b = bytePointer;
  }
  
  public byte[] get_bytes() {
    return this.b;
  }
  
  public String toString() {
    String s = "";
    for(int i=0; i<this.b.length; i++){
      s += " " + b[i];
    }
    return s;
  }
  
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // PUBLIC STATIC METHODS
  ///////////////////////////////////////////////////////////////////////////////////////
  
  public static void encode_int8( byte value , byte[] b , int index ) {
    b[index] = value;
  }
  
  public static byte decode_int8( byte[] b , int index ) {
    return b[index];
  }
  
  public static void encode_int16( short value , byte[] b , int index ) {
    b[index] = (byte)value;
    b[index+1] = (byte)( value >> 8 );
  }
  
  public static short decode_int16( byte[] b , int index ) {
    return (short)( ( b[index+1] << 8 ) | (b[index] & 0xFF) );
  }
  
  public static void encode_int32( int value , byte[] b , int index ) {
    b[index] = (byte)value;
    b[index+1] = (byte)( value >> 8 );
    b[index+2] = (byte)( value >> 16 );
    b[index+3] = (byte)( value >> 24 );
  }
  
  public static int decode_int32( byte[] b , int index ) {
    return ( ( b[index+3] << 24 ) | ( (b[index+2] & 0xFF) << 16 ) | ( (b[index+1] & 0xFF) << 8 ) | (b[index] & 0xFF) );
  }
  
  public static void encode_int64( long value , byte[] b , int index ) {
    b[index] = (byte)value;
    b[index+1] = (byte)( value >> 8 );
    b[index+2] = (byte)( value >> 16 );
    b[index+3] = (byte)( value >> 24 );
    b[index+4] = (byte)( value >> 32 );
    b[index+5] = (byte)( value >> 40 );
    b[index+6] = (byte)( value >> 48 );
    b[index+7] = (byte)( value >> 56 );
  }
  
  public static long decode_int64( byte[] b , int index ) {
    return ( ( (b[index+7] & 0xFFL) << 56 ) | ( (b[index+6] & 0xFFL) << 48 ) | ( (b[index+5] & 0xFFL) << 40 ) | ( (b[index+4] & 0xFFL) << 32 ) | ( (b[index+3] & 0xFFL) << 24 ) | ( (b[index+2] & 0xFFL) << 16 ) | ( (b[index+1] & 0xFFL) << 8 ) | (b[index] & 0xFFL) );
  }
  
  public static void encode_float( float value , byte[] b , int index ) {
    int theBits = Float.floatToIntBits( value );
    b[index] = (byte)theBits;
    b[index+1] = (byte)( theBits >> 8 );
    b[index+2] = (byte)( theBits >> 16 );
    b[index+3] = (byte)( theBits >> 24 );
  }
  
  public static float decode_float( byte[] b , int index ) {
    return Float.intBitsToFloat( ( (b[index+3] & 0xFF) << 24 ) | ( (b[index+2] & 0xFF) << 16 ) | ( (b[index+1] & 0xFF) << 8 ) | (b[index] & 0xFF) );
  }
  
}