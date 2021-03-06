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


public class IPM_AdafruitIMU9dof
  extends IPM_MARG {
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // CONSTRUCTORS
  ///////////////////////////////////////////////////////////////////////////////////////
  
  public IPM_AdafruitIMU9dof( byte ID ) {
    this.b = new byte[NON_MEASUREMENT_BYTES+21];  // 10*int16 + 1*int8
    this.b[0] = 3;  // information packet ID
    this.b[1] = ID;  // sensor ID
  }
  
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // PUBLIC METHODS
  ///////////////////////////////////////////////////////////////////////////////////////
  
  public void set_a( short ax , short ay , short az ) {
    IPM.encode_int16( ax , this.b , 2 );
    IPM.encode_int16( ay , this.b , 4 );
    IPM.encode_int16( az , this.b , 6 );
  }
  
  public void set_m( short mx , short my , short mz ) {
    IPM.encode_int16( mx , this.b , 8 );
    IPM.encode_int16( my , this.b , 10 );
    IPM.encode_int16( mz , this.b , 12 );
  }
  
  public void set_T( short T ) {
    IPM.encode_int16( T , this.b , 14 );
  }
  
  public void set_w( short wx , short wy , short wz ) {
    IPM.encode_int16( wx , this.b , 16 );
    IPM.encode_int16( wy , this.b , 18 );
    IPM.encode_int16( wz , this.b , 20 );
  }
  
  public void set_Tw( byte T ) {
    IPM.encode_int8( T , this.b , 22 );
  }
  
  // IPM_MARG implementation
  
  public double[] get_a() {
    double[] am = new double[3];
    am[0] = IPM.decode_int16( this.b , 2 );
    am[1] = IPM.decode_int16( this.b , 4 );
    am[2] = IPM.decode_int16( this.b , 6 );
    return am;
  }
  
  public double get_Ta() {
    return IPM.decode_int16( this.b , 14 );
  }
  
  public double[] get_m() {
    double[] mm = new double[3];
    mm[0] = IPM.decode_int16( this.b , 8 );
    mm[1] = IPM.decode_int16( this.b , 10 );
    mm[2] = IPM.decode_int16( this.b , 12 );
    return mm;
  }
  
  public double get_Tm() {
    return IPM.decode_int16( this.b , 14 );
  }
  
  public double[] get_w() {
    double[] wm = new double[3];
    wm[0] = IPM.decode_int16( this.b , 16 );
    wm[1] = IPM.decode_int16( this.b , 18 );
    wm[2] = IPM.decode_int16( this.b , 20 );
    return wm;
  }
  
  public double get_Tw() {
    return IPM.decode_int8( this.b , 22 );
  }
  
  // toString
  
  public String toString() {
    double[] am = this.get_a();
    double[] mm = this.get_m();
    double[] wm = this.get_w();
    return String.format( "%6d %6d %6d  %6d %6d %6d  %6d  %6d %6d %6d  %6d" ,
                          (int)am[0] , (int)am[1] , (int)am[2] ,
                          (int)mm[0] , (int)mm[1] , (int)mm[2] ,
                          (int)this.get_Ta() ,
                          (int)wm[0] , (int)wm[1] , (int)wm[2] ,
                          (int)this.get_Tw() );
  }
  
}