using UnityEngine;

namespace Kalmanspace
{
    public class Kalman1D {
        public float X;    // Estado: valor estimado (µT)
        public float V;    // Estado: velocidad estimada (µT/s)

        private float P11, P12, P21, P22;  // Covarianza del estado 2x2

        private readonly float Q = 0.01f;  // Varianza del proceso (ajustable)
        private readonly float R = 0.000196f; // Varianza del sensor (0.014^2 µT²)

        public Kalman1D(float initialValue) {
            X = initialValue;
            V = 0f;
            P11 = P22 = 1f;
            P12 = P21 = 0f;
        }

        // Predice el estado hacia adelante sin medir
        public void Predict(float dt) {
            // Modelo lineal: X += V*dt
            X += V * dt;

            // Actualizar covarianza
            float dt2 = dt * dt;
            float newP11 = P11 + dt * (P21 + P12 + dt * P22) + Q;
            float newP12 = P12 + dt * P22;
            float newP21 = P21 + dt * P22;
            float newP22 = P22 + Q;

            P11 = newP11;
            P12 = newP12;
            P21 = newP21;
            P22 = newP22;
        }

        // Corrige el estado con una nueva medición del magnetómetro
        public void Update(float measurement) {
            // Ganancia de Kalman
            float S = P11 + R;
            float K1 = P11 / S;
            float K2 = P21 / S;

            // Error de medición
            float y = measurement - X;

            // Actualizar estado
            X += K1 * y;
            V += K2 * y;

            // Actualizar covarianza
            float P11_ = P11, P12_ = P12, P21_ = P21, P22_ = P22;
            P11 = P11_ - K1 * P11_;
            P12 = P12_ - K1 * P12_;
            P21 = P21_ - K2 * P11_;
            P22 = P22_ - K2 * P12_;
        }

        // Llamado cuando hay una nueva medición
        public void Step(float dt, float measurement) {
            Predict(dt);
            Update(measurement);
        }

        // Llamado si NO hay medición (solo predicción)
        public void Step(float dt) {
            Predict(dt);
        }
    }


    public class KalmanVector3 {
        private Kalman1D kalmanX;
        private Kalman1D kalmanY;
        private Kalman1D kalmanZ;

        public KalmanVector3(Vector3 initialValue) {
            kalmanX = new Kalman1D(initialValue.x);
            kalmanY = new Kalman1D(initialValue.y);
            kalmanZ = new Kalman1D(initialValue.z);
        }

        // Predice hacia adelante (cuando no hay nueva medición)
        public void Predict(float dt) {
            kalmanX.Predict(dt);
            kalmanY.Predict(dt);
            kalmanZ.Predict(dt);
        }

        // Corrige con una nueva medición
        public void Update(Vector3 measurement) {
            kalmanX.Update(measurement.x);
            kalmanY.Update(measurement.y);
            kalmanZ.Update(measurement.z);
        }

        // Paso completo: predicción + corrección
        public void Step(float dt, Vector3 measurement) {
            kalmanX.Step(dt, measurement.x);
            kalmanY.Step(dt, measurement.y);
            kalmanZ.Step(dt, measurement.z);
        }

        // Paso solo con predicción (sin medición)
        public void Step(float dt) {
            kalmanX.Step(dt);
            kalmanY.Step(dt);
            kalmanZ.Step(dt);
        }

        // Obtener el estado estimado actual
        public Vector3 GetEstimate() {
            return new Vector3(kalmanX.X, kalmanY.X, kalmanZ.X);
        }

        // Obtener velocidades estimadas (opcional)
        public Vector3 GetVelocity() {
            return new Vector3(kalmanX.V, kalmanY.V, kalmanZ.V);
        }
    }
}
