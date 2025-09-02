import * as ROSLIB from 'roslib';

let ros: ROSLIB.Ros | null = null;

// Publishers
let pubCordRos: ROSLIB.Topic | null = null;
let pubCordDy: ROSLIB.Topic | null = null;

// Subscriber
let subPosDy: ROSLIB.Topic | null = null;

/**
 * Conecta a ROS
 */
export function conectarRos(): void {
  if (!ros) {
    ros = new ROSLIB.Ros({
      url: "ws://localhost:9090",
    });

    ros.on("connection", () => {
      console.log("✅ Conectado a ROS");
    });

    ros.on("error", (error) => {
      console.error("❌ Error de conexión:", error);
    });

    ros.on("close", () => {
      console.warn("🔌 Conexión cerrada");
    });

    // Inicializar tópicos
    pubCordRos = new ROSLIB.Topic({
      ros,
      name: "/cord_ros",
      messageType: "std_msgs/Int16MultiArray",
    });

    pubCordDy = new ROSLIB.Topic({
      ros,
      name: "/cord_dy",
      messageType: "std_msgs/Int16MultiArray",
    });

    subPosDy = new ROSLIB.Topic({
      ros,
      name: "/pos_dy",
      messageType: "std_msgs/Int16MultiArray",
    });
  }
}

/**
 * Publica en /cord_ros
 */
export function publicarCordRos(datos: number[]): void {
  if (!pubCordRos) return console.warn("ROS no conectado");

  const msg = new ROSLIB.Message({ data: datos });
  pubCordRos.publish(msg);
}

/**
 * Publica en /cord_dy
 */
export function publicarCordDy(datos: number[]): void {
  if (!pubCordDy) return console.warn("ROS no conectado");

  const msg = new ROSLIB.Message({ data: datos });
  pubCordDy.publish(msg);
}

/**
 * Se suscribe a /pos_dy con una función de callback
 */
export function suscribirsePosDy(callback: (datos: number[]) => void): void {
  if (!subPosDy) return console.warn("ROS no conectado");

  subPosDy.subscribe((msg: ROSLIB.Message) => {
    const datos = (msg as any).data as number[];
    callback(datos);
  });
}
