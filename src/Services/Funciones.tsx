import {initRos, pubTopic, subTopic, unsubTopic } from './RosService2';


//////////////////////////    Section1     /////////////////////////

export function cargarPuntosDB() {
  const msg = {
    orden: ["subir_db"],
    coordenadas: []
  };

  console.log(" Enviando a /orden_web:", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

export function nombrePuntoDB(callback: (msg: any) => void) {
  subTopic("lista_puntosdb", callback);

  // ❌ Luego analizar si conviene cerrarlo o cuando hacerlo
}

export function elimiarPuntoDB(nombrePunto:string) {
  const confirmar = confirm(`¿Estás seguro de querer eliminar el Punto "${nombrePunto}"?`);
  const msg = {
    orden: ['eliminar', nombrePunto],
    coordenadas: []
  };

  console.log(" Enviando a /orden_web: ELIMIAR", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

export function agregarPuntoDB(nombrePunto:string, coord:number[]) {
  const msg = {
    orden: ['agregar', nombrePunto],
    coordenadas: [coord]
  };

  console.log(" Enviando a /orden_web: agregando punto", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

//////////////////////////    FIN Section1     /////////////////////////


//////////////////////////    Section2     ////////////////////////////


export function correrTAngular( coord:number[]) {
  const msg = {
    data: [coord]
  };

  console.log(" Enviando a /cord_ros: correr angulos", msg);

  // Publica en ROS
  pubTopic('cord_ros', msg);
}

export function correrTCartesiano( coord:number[]) {
  const msg = {
    data: [coord]
  };

  console.log(" Enviando a /cord_ros: correr cartesianos", msg);

  // Publica en ROS
  pubTopic('cord_ros', msg);
}

export function escucharPuntoDB(callback: (msg: any) => void) {
  subTopic("puntoDB", callback);

  // ❌ Luego analizar si conviene cerrarlo o cuando hacerlo
}

//////////////////////////    FIN Section2     /////////////////////////

export function agregarPuntoRutina(nombrePunto:string,plan:string, coord:number[]) {
  const msg = {
    orden: ['addR', nombrePunto,plan],
    coordenadas: [coord]
  };

  console.log(" Enviando a /orden_web: agregando punto rutina", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

//////////////////////////    Section3     ////////////////////////////


export function mostrarPuntoRutina(callback: (msg: any) => void) {
  subTopic("puntoRutina", callback);
}

export function elimiarPuntoRutina(id:number) {
  const msg = {
    orden: ['eliminarRut', id],
    coordenadas: []
  };

  console.log("del", id);

  // Publica en ROS
  pubTopic('orden_web', msg);
}