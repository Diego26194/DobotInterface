// rosService.ts
import ROSLIB from 'roslib';

// Instancia de ROS y tópicos compartidos
let ros: ROSLIB.Ros;
const topics: { [key: string]: ROSLIB.Topic } = {};



// ✅ Inicializa ROS y crea los tópicos necesarios
export function initRos() {
  ros = new ROSLIB.Ros({
    url: 'ws://' + window.location.hostname + ':9090',
  });

  ros.on('connection', () => console.log('Conectado a ROS'));
  ros.on('error', (error) => console.error('Error con ROS:', error));
  ros.on('close', () => console.log('Conexión cerrada con ROS'));

  // Inicializar tópicos
  topics['cord_ros'] = new ROSLIB.Topic({
    ros,
    name: '/cord_ros',
    messageType: 'cobot_controladores/punto_correr',
  });

  topics['orden_web'] = new ROSLIB.Topic({
    ros,
    name: '/orden_web',
    messageType: 'cobot_controladores/punto_web',
  });

  topics['puntodb'] = new ROSLIB.Topic({
    ros,
    name: '/puntodb',
    messageType: 'cobot_controladores/punto_web',
  });

  topics['puntoRutina'] = new ROSLIB.Topic({
    ros,
    name: '/puntoRutina',
    messageType: 'cobot_controladores/punto_web',
  });

  topics['lista_puntosdb'] = new ROSLIB.Topic({
    ros,
    name: '/lista_puntosdb',
    messageType: 'cobot_controladores/nombresPuntos',
  });

  topics['lista_rutinasdb'] = new ROSLIB.Topic({
    ros,
    name: '/lista_rutinasdb',
    messageType: 'cobot_controladores/nombresPuntos',
  });

  topics['informe_web'] = new ROSLIB.Topic({
    ros,
    name: '/informe_web',
    messageType: 'std_msgs/String',
  });

  topics['pos_dy'] = new ROSLIB.Topic({
    ros,
    name: '/pos_dy',
    messageType: 'std_msgs/Int16MultiArray',
  });

  topics['pos_real'] = new ROSLIB.Topic({
    ros,
    name: '/pos_real',
    messageType: 'cobot_controladores/punto_real',
  });

  topics['p_dy'] = new ROSLIB.Topic({
    ros,
    name: '/p_dy',
    messageType: 'std_msgs/Int16MultiArray',
  });

  topics['modo_actuar'] = new ROSLIB.Topic({
    ros,
    name: '/modo_actuar',
    messageType: 'std_msgs/Bool',
  });

  topics['tipo_modo_lectura'] = new ROSLIB.Topic({
    ros,
    name: '/tipo_modo_lectura',
    messageType: 'std_msgs/Bool',
  });

  topics['rutina'] = new ROSLIB.Topic({
    ros,
    name: '/rutina',
    messageType: 'std_msgs/Bool',
  });

  ////////////// Eliminar-Prueba de emulacion arduino///////
  topics['cord_dy'] = new ROSLIB.Topic({
    ros,
    name: '/cord_dy',
    messageType: 'std_msgs/Int16MultiArray',
  });
  
  /*
// Si se desconecta, reintenta
ros.on("close", () => {
  console.log("Conexión perdida, reintentando...");
  setTimeout(() => {
    ros.connect("ws://localhost:9090");
  }, 3000);
*/
}



//Publicacion Generica

export function pubTopic(topicName: string, message: any) {
  if (!topics[topicName]) {
    console.error(`El tópico ${topicName} no está inicializado.`);
    return;
  }
  const rosMsg = new ROSLIB.Message(message);
  topics[topicName].publish(rosMsg);
}


//Subscripcion Generica

export function subTopic(topicName: string, callback: (msg: any) => void) {
  if (!topics[topicName]) {
    console.error(`El tópico ${topicName} no está inicializado.`);
    return;
  }
  topics[topicName].subscribe((msg: any) => {
    callback(msg);
  });
}

//Cancelar subscripcion

export function unsubTopic(topicName: string) {
  if (topics[topicName]) {
    topics[topicName].unsubscribe();
  }
}


