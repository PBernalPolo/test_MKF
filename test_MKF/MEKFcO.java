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


public class MEKFcO extends MEKF {
  
  // PROTECTED METHODS
  
  // Method: fC2M
  // defines the map from the chart points, to the manifold points (through the delta quaternion)
  // inputs:
  //  e: point of the Euclidean space that we want to map to a unit quaternion
  // outputs:
  //  delta: quaternion mapped with the e point
  protected void fC2M( double[] e , double[] delta ){
    // delta from the chart definition: Orthographic
    double enorm = Math.sqrt( e[0]*e[0] + e[1]*e[1] + e[2]*e[2] );
    double epsilon = 1.0e-3;
    if( enorm > 2.0-epsilon ){
      double aux = (2.0-epsilon)/enorm;
      e[0] *= aux;
      e[1] *= aux;
      e[2] *= aux;
      enorm = 2.0-epsilon;
    }
    delta[0] = Math.sqrt(1.0-0.25*enorm*enorm);
    delta[1] = 0.5*e[0];
    delta[2] = 0.5*e[1];
    delta[3] = 0.5*e[2];
    
    return;
  }
  
  // Method: chartUpdateMatrix
  // this function defines the transformation on the covariance matrix when it is 
  // redefined from the chart centered in q quaternion, to the chart centered in
  // p quaternion, being them related by  p = q * delta
  // inputs:
  //  delta: quaternion used to update the quaternion estimation
  // outputs:
  //  G: transformation matrix to update the covariance matrix
  protected void chartUpdateMatrix( double[] delta , double[] G ){
    G[0] = delta[0];     G[3] = delta[3];     G[6] = -delta[2];
    G[1] = -delta[3];    G[4] = delta[0];     G[7] = delta[1];
    G[2] = delta[2];     G[5] = -delta[1];    G[8] = delta[0];
    
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++) G[i+j*3] += delta[i+1]*delta[j+1]/delta[0];
    }
    
    return;
  }
  
}