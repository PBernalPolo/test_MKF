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


public abstract class MUKF extends OrientationEstimator {
  
  // PUBLIC METHODS
  
  MUKF(){
    super();
    this.q0[0] = 1.0;   this.q0[1] = 0.0;   this.q0[2] = 0.0;   this.q0[3] = 0.0;
    this.e[0] = 0.0;   this.e[1] = 0.0;   this.e[2] = 0.0;
  }
  
  public void set_q( double[] qIn ){
    super.set_q( qIn );
    for(int i=0; i<4; i++) this.q0[i] = qIn[i];
    this.e[0] = 0.0;   this.e[1] = 0.0;   this.e[2] = 0.0;
  }
  
  public void reset_orientation(){
    super.reset_orientation();
    this.q0[0] = 1.0;   this.q0[1] = 0.0;   this.q0[2] = 0.0;   this.q0[3] = 0.0;
    this.e[0] = 0.0;   this.e[1] = 0.0;   this.e[2] = 0.0;
  }
  
  // Method: updateIMU
  // method used to update the state information through an IMU measurement
  // inputs:
  //  am: measured acceleration (g)
  //  wm: measured angular velocity (rad/s)
  //  dt: time step from the last update (s)
  // outputs:
  public void updateIMU( double[] am , double[] wm , double dt ){
    // we define the extended covariance matrix
    double[] Pe = new double[144];  // Pe is 12x12
    for(int k=0; k<144; k++) Pe[k] = 0.0;
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++) Pe[i+j*12] = this.P[i+j*6];
    }
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++) Pe[78+i+j*12] = this.Qw[i+j*3];
    }
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++) Pe[117+i+j*12] = this.Qa[i+j*3];
    }
    
    // we get the square-root of the matrix using the Cholesky factorization
    MUKF.Cholesky( Pe , 12 );
    // we define the weight for the 0 sigma point (W0 must be in [0,1])
    //double W0 = 1.0/25.0;
    // the weights for the rest of sigma points are defined by the first one
    double Wi = (1.0-this.W0)/(2.0*12);
    // the factor for the square root, so P = sum(W_k*sigma_k)
    double alpha = 1.0/Math.sqrt(2.0*Wi);
    for(int k=0; k<144; k++) Pe[k] *= alpha;
    
    // we define and initialize the sigma points (state and measure)
    double[][] X = new double[25][13];
    for(int j=0; j<25; j++){
      for(int i=0; i<13; i++) X[j][i] = 0.0;
    }
    double[][] Y = new double[25][6];
    for(int j=0; j<25; j++){
      for(int i=0; i<6; i++) Y[j][i] = 0.0;
    }
    
    // first we set the mean value
    for(int i=0; i<4; i++) X[0][i] = this.q[i];
    for(int i=0; i<3; i++) X[0][i+4] = this.w[i];
    for(int i=0; i<6; i++) X[0][i+7] = 0.0;
    // we can test if the Chart update is good or not by uncommenting the next lines
    if( !this.chartUpdate ){
      for(int i=0; i<4; i++) this.q0[i] = this.q[i];
      for(int i=0; i<3; i++) this.e[i] = 0.0;
    }
    // second we generate the +sigma points from the P matrix
    for(int j=0; j<12; j++){
      // we do this because P is expressed in the q0 chart, but we need to
      // express it in the q chart for the next time step
      //   first we compute the point in the chart
      double[] eP = { this.e[0]+Pe[0+j*12] , this.e[1]+Pe[1+j*12] , this.e[2]+Pe[2+j*12] };
      //   we get the point in the manifold
      this.fC2M( X[j+1] , this.q0 , eP );
      // we set the angular velocity
      for(int i=3; i<6; i++) X[j+1][i+1] = this.w[i-3] + Pe[i+j*12];
      for(int i=6; i<12; i++) X[j+1][i+1] = Pe[i+j*12];
    }
    // third we generate the -sigma points from the P matrix
    for(int j=0; j<12; j++){
      // we do this because P is expressed in the q0 chart, but we need to
      // express it in the q chart for the next time step
      //   first we compute the point in the chart
      double[] eP = { this.e[0]-Pe[0+j*12] , this.e[1]-Pe[1+j*12] , this.e[2]-Pe[2+j*12] };
      //   we get the point in the manifold
      this.fC2M( X[j+13] , this.q0 , eP );
      // we set the angular velocity
      for(int i=3; i<6; i++) X[j+13][i+1] = this.w[i-3] - Pe[i+j*12];
      for(int i=6; i<12; i++) X[j+13][i+1] = -Pe[i+j*12];
    }
    
    // we compute the predictions
    for(int j=0; j<25; j++){
      MUKF.statePrediction( X[j] , dt );
      // we make sure that all quaternions are in the same hemisphere
      double prod = 0.0;
      for(int i=0; i<4; i++) prod += X[0][i]*X[j][i];
      if( prod < 0.0 ){
        for(int i=0; i<4; i++) X[j][i] = -X[j][i];
      }
      //if( X[j][0] < 0.0 ) for(int i=0; i<4; i++) X[j][i] = -X[j][i];  // this is an alternative
      MUKF.IMU_MeasurementPrediction( Y[j] , X[j] );
    }
    
    // we compute the means
    double[] xmean = new double[7];
    double[] ymean = new double[6];
    for(int i=0; i<7; i++) xmean[i] = this.W0*X[0][i];
    for(int i=0; i<6; i++) ymean[i] = this.W0*Y[0][i];
    for(int j=1; j<25; j++){
      for(int i=0; i<7; i++) xmean[i] += Wi*X[j][i];
      for(int i=0; i<6; i++) ymean[i] += Wi*Y[j][i];
    }
    double qmeanNorm = Math.sqrt( xmean[0]*xmean[0] + xmean[1]*xmean[1] + xmean[2]*xmean[2] + xmean[3]*xmean[3] );
    for(int i=0; i<4; i++) xmean[i] /= qmeanNorm;
    
    // we compute the covariance matrices
    double[] Pxx = new double[6*6];
    double[] Pxy = new double[6*6];
    double[] Pyy = new double[6*6];
    double[] dX = new double[6];
    //   first we add the 0 contribution
    this.fM2C( dX , xmean , X[0] );
    for(int i=3; i<6; i++) dX[i] = X[0][i+1]-xmean[i+1];
    double[] dY = new double[6];
    for(int i=0; i<6; i++) dY[i] = Y[0][i]-ymean[i];
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        Pxx[i+j*6] = W0*dX[i]*dX[j];
        Pxy[i*6+j] = W0*dX[i]*dY[j];
        Pyy[i+j*6] = W0*dY[i]*dY[j];
      }
    }
    //   then the rest
    for(int k=1; k<25; k++){
      this.fM2C( dX , xmean , X[k] );
      for(int i=3; i<6; i++) dX[i] = X[k][i+1]-xmean[i+1];
      for(int i=0; i<6; i++) dY[i] = Y[k][i]-ymean[i];
      for(int i=0; i<6; i++){
        for(int j=0; j<6; j++){
          Pxx[i+j*6] += Wi*dX[i]*dX[j];
          Pxy[i*6+j] += Wi*dX[i]*dY[j];
          Pyy[i+j*6] += Wi*dY[i]*dY[j];
        }
      }
    }
    //   finally we add the noise (the linear part)
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
        Pyy[i+j*6] += this.Ra[i+j*3];
        Pyy[i+3+(j+3)*6] += this.Rw[i+j*3];
      }
    }
    
    // we save Pyy in other matrix because solve() will overwrite it
    for(int k=0; k<36; k++) this.P[k] = Pyy[k];
    
    // now we can compute the gain ( K*Pyy = Pxy )
    MUKF.solve( this.P , Pxy );  // now K is stored in Pxy
    
    // and update the state in the chart
    double[] dy = { am[0]-ymean[0] , am[1]-ymean[1] , am[2]-ymean[2] , wm[0]-ymean[3] , wm[1]-ymean[4] , wm[2]-ymean[5] };
    
    double[] dx = new double[6];
    for(int i=0; i<6; i++){
      double sum = 0.0;
      for(int j=0; j<6; j++) sum += Pxy[i*6+j]*dy[j];
      dx[i] = sum;
    }
    
    // this update takes place in the chart centered in xmean
    for(int i=0; i<4; i++) this.q0[i] = xmean[i];
    for(int i=0; i<3; i++) this.e[i] = dx[i];
    
    // the updated point in the chart is mapped to a quaternion
    double[] delta = new double[4];
    this.fC2M( this.q , this.q0 , this.e );
    // and the angular velocity is updated in the usual way
    this.w[0] = xmean[4] + dx[3];
    this.w[1] = xmean[5] + dx[4];
    this.w[2] = xmean[6] + dx[5];
    
    // the covariance matrix is updated in the chart centered in q0 ( P = Pxx - K*Pyy*K^T )
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        double sum = 0.0;
        for(int k=0; k<6; k++) sum += Pyy[i+k*6]*Pxy[j*6+k];
        this.P[i+j*6] = sum;
      }
    }
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        double sum = 0.0;
        for(int k=0; k<6; k++) sum += Pxy[i*6+k]*this.P[k+j*6];
        Pxx[i+j*6] -= sum;
      }
    }
    
    // we avoid numerical instabilities
    double qnorm = Math.sqrt( this.q[0]*this.q[0] + this.q[1]*this.q[1] + this.q[2]*this.q[2] + this.q[3]*this.q[3] );
    for(int i=0; i<4; i++) this.q[i] /= qnorm;
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++) this.P[i+j*6] = 0.5*( Pxx[i+j*6] + Pxx[j+i*6] );
    }
    
    // this covariance matrix is expressed in the q0 chart
    // we will have to update it to the new q chart
    // that is why we do what we do at the begining  
    
    return;
  }
  
  
  // ABSTRACT METHODS
  
  // Method: fM2C
  // defines the map from the manifold points, to the chart points
  // inputs:
  //  qm: mean quaternion of the distribution (is mapped with the origin of the chart)
  //  q: quaternion that we want to map with a point in the chart
  // outputs:
  //  e: point in the chart mapped with the q quaternion
  protected abstract void fM2C( double[] e , double[] qm , double[] q );
  
  // Method: fC2M
  // defines the map from the chart points, to the manifold points
  // inputs:
  //  qm: mean quaternion of the distribution (it is mapped with the origin of the chart)
  //  e: point of the chart that we want to map to a unit quaternion in the manifold
  // outputs:
  //  q: quaternion in the manifold mapped with the e point in the chart
  protected abstract void fC2M( double[] q , double[] qm , double[] e );
  
  
  // PRIVATE VARIABLES
  // quaternion used for the last update (q1,q2,q3,q4)=(qx,qy,qz,qw)
  // (rotation that transform vectors from the sensor reference frame, to the external reference frame)
  double[] q0 = new double[4];
  // last updated point in the chart
  double[] e = new double[3];
  
  
  // PRIVATE STATIC METHODS
  
  // Method: statePrediction
  // this method predicts the state given the previous state, and the time increment
  // inputs:
  //  x: previous state (q,w,n,a)
  //  dt: time step
  // outputs:
  //  xp: predicted state (qp,wp,np,ap)
  private static void statePrediction( double[] x , double dt ){
    // first we predict the angular velocity
    double[] wp = { x[4]+x[7]*dt , x[5]+x[8]*dt , x[6]+x[9]*dt };
    // angular velocity norm computation
    double wnorm = Math.sqrt( wp[0]*wp[0] + wp[1]*wp[1] + wp[2]*wp[2] );
    // we compute qw
    double[] qw = new double[4];
    if( wnorm != 0.0 ){
      double wdt05 = 0.5*wnorm*dt;
      double swdt = Math.sin(wdt05)/wnorm;
      qw[0] = Math.cos(wdt05);
      qw[1] = wp[0]*swdt;
      qw[2] = wp[1]*swdt;
      qw[3] = wp[2]*swdt;
    }else{
      qw[0] = 1.0;
      qw[1] = 0.0;
      qw[2] = 0.0;
      qw[3] = 0.0;
    }
    // we compute the predicted state (q*qw,w)
    double[] qp = new double[4];
    qp[0] = x[0]*qw[0] - x[1]*qw[1] - x[2]*qw[2] - x[3]*qw[3];
    qp[1] = x[0]*qw[1]  +  qw[0]*x[1]  +  x[2]*qw[3] - x[3]*qw[2];
    qp[2] = x[0]*qw[2]  +  qw[0]*x[2]  +  x[3]*qw[1] - x[1]*qw[3];
    qp[3] = x[0]*qw[3]  +  qw[0]*x[3]  +  x[1]*qw[2] - x[2]*qw[1];
    // we overwrite the predicted state in the current state
    for(int i=0; i<4; i++) x[i] = qp[i];
    for(int i=0; i<3; i++) x[i+4] = wp[i];
    
    return;
  }
  
  // Function: IMU_MeasurementPrediction
  // this method predicts the measurement given a state
  // inputs:
  //  xp: state for which the measure is to be predicted
  // outputs:
  //  yp: predicted measurement
  private static void IMU_MeasurementPrediction( double[] y , double[] x ){
    // the predicted acceleration measurement will be the gravity vector measured
    // in the sensor frame: g = (R^T)*[a-(0,0,-1)]
    //  first we compute the rotation matrix
    double[] RT = new double[9];
    RT[0] = -x[2]*x[2]-x[3]*x[3];    RT[3] = x[1]*x[2]+x[3]*x[0];     RT[6] = x[1]*x[3]-x[2]*x[0];
    RT[1] = x[1]*x[2]-x[3]*x[0];     RT[4] = -x[1]*x[1]-x[3]*x[3];    RT[7] = x[2]*x[3]+x[1]*x[0];
    RT[2] = x[1]*x[3]+x[2]*x[0];     RT[5] = x[2]*x[3]-x[1]*x[0];     RT[8] = -x[1]*x[1]-x[2]*x[2];
    
    RT[0] += RT[0] + 1.0;    RT[3] += RT[3];          RT[6] += RT[6];
    RT[1] += RT[1];          RT[4] += RT[4] + 1.0;    RT[7] += RT[7];
    RT[2] += RT[2];          RT[5] += RT[5];          RT[8] += RT[8] + 1.0;
    
    double[] ag = { x[10] , x[11] , x[12]+1.0 };
    for(int i=0; i<3; i++){
      double sum = 0.0;
      for(int j=0; j<3; j++) sum += RT[i+j*3]*ag[j];
      y[i] = sum;
    }
    
    // the predicted measurement for the angular velocity will be itself
    y[3] = x[4];
    y[4] = x[5];
    y[5] = x[6];
    
    return;
  }
  
  // Method: Cholesky
  // performs the Cholesky decomposition of a positive definite matrix ( S = L*L' )
  // inputs:
  //  S: positive definite matrix to be decomposed (must be stored by columns)
  //  n: number of rows or column of the square matrix S (nxn)
  // outputs:
  //  S: the lower triangular matrix L (nxn) is overwritten in S (is stored by columns)
  private static void Cholesky( double[] S , int n ){
    // for each column
    for(int j=0; j<n; j++){
      double sum = 0.0;  //sum for the diagonal term
      // we first fill with 0.0 until diagonal
      for(int i=0; i<j; i++){
        S[i+j*n] = 0.0;
        //we can compute this sum at the same time
        sum += S[j+i*n]*S[j+i*n];
      }
      // now we compute the diagonal term
      S[j*(n+1)] = Math.sqrt( S[j*(n+1)] - sum ); //S[j+j*m] = sqrt( S[j+j*m] - sum );
      // finally we compute the terms below the diagonal
      for(int i=j+1; i<n; i++){
        //first the sum
        sum = 0.0;
        for(char k=0; k<j; k++){
          sum += S[i+k*n]*S[j+k*n];
        }
        //after the non-diagonal term
        S[i+j*n] = ( S[i+j*n] - sum )/S[j*(n+1)];
      }
    }//end j
    
    return;
  }
  
  // Method: solve
  // solves the system of linear equations  K*S = M  for K
  // inputs:
  //  S: 6x6 positive definite matrix stored by columns
  //  M: 6x6 matrix stored by rows
  // outputs:
  //  M: K (6x6) is stored by rows in the M memory space
  private static void solve( double[] S , double[] M ){
    // we first compute the Cholesky decomposition for transform the system from  K*S = M  into K*L*L' = M
    MUKF.Cholesky( S , 6 );
    
    double[] y = new double[6];
    // then we take each pair of rows of K and M independently
    for(int i=0; i<6; i++){
      // first we solve (y*L' = M)
      for(int j=0; j<6; j++){
        double sum = 0.0;
        for(int k=0; k<j; k++){
          sum += y[k]*S[j+k*6];
        }
        y[j] = ( M[i*6+j] - sum )/S[j*7];
      }
      // now we solve (Ki*L = y)
      for(int j=5; j>-1; j--){
        double sum = 0.0;
        for(int k=j+1; k<6; k++){
          sum += M[i*6+k]*S[k+j*6];
        }
        M[i*6+j] = ( y[j] - sum )/S[j*7];
      }
    }
    
    return;
  }
  
  
}