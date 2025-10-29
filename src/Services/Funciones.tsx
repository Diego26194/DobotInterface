import {initRos, pubTopic, subTopic, unsubTopic } from './RosService2';


//////////////////////////    Section1     /////////////////////////

//Envia Orden para traer puntos desde la libreria de punto (Refrescar)
export function cargarPuntosDB() {
  const msg = {
    orden: ["subir_db"],
    coordenadas: []
  };

  console.log(" Enviando a /orden_web:", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

//Agrega Punto a Libreria de Puntos (Puesto ahora en Section2)
export function agregarPuntoDB(nombrePunto:string, coord:number[]) {
  const msg = {
    orden: ['agregar', nombrePunto],
    coordenadas: coord
  };

  console.log(" Enviando a /orden_web: agregando punto", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

// Elimina Punto seleccionado
export function elimiarPuntoDB(nombrePunto:string) {
  if (nombrePunto){
    const confirmar = confirm(`¿Estás seguro de querer eliminar el Punto "${nombrePunto}"?`);
    if (confirmar){
      const msg = {
        orden: ['eliminar', nombrePunto],
        coordenadas: []
      };

      console.log(" Enviando a /orden_web: ELIMIAR", msg);

      // Publica en ROS
      pubTopic('orden_web', msg);
    } 
  }    
}

// Mostrar punto
export function mostrarPuntoDB(nombrePunto:string) {
  if (nombrePunto){
    const msg = {
      orden: ['ver', nombrePunto],
      coordenadas: []
    };
    console.log(" Enviando a /orden_web: ELIMIAR", msg);

    // Publica en ROS
    pubTopic('orden_web', msg);
  } 
}

// carga rutina en "rutina_actual"
export function cargarRutinaRA(nombreRutina:string) {
  if (nombreRutina){
    const msg = {
      orden: ['cargarRRA', nombreRutina],
      coordenadas: []
    };
    console.log(" Enviando a /orden_web:", msg);

    // Publica en ROS
    pubTopic('orden_web', msg);
  } 
}

//Resibe lista con todos los puntos de la libreria
export function nombrePuntoDB(callback: (msg: any) => void) {
  subTopic("lista_puntosdb", callback);
  
}

//Resibe lista con todos los puntos de la libreria
export function nombreRutinaDB(callback: (msg: any) => void) {
  subTopic("lista_rutinasdb", callback);

}


export function elimiarRutinaDB(nombreRutina:string) {
  console.log("a",nombreRutina,"no");
  if (nombreRutina){
    const confirmar = confirm(`¿Estás seguro de querer eliminar esta Rutina "${nombreRutina}"?`);
    if (confirmar){
      const msg = {
        orden: ['eliminarRutina', nombreRutina],
        coordenadas: []
      };
      console.log(" Enviando a /orden_web: ELIMIAR", msg);

      // Publica en ROS
      pubTopic('orden_web', msg);
    } 
  }  
}


export function agregarRutinaTabla(nombreRutina:string) { //agregar posicion despues
  console.log("a",nombreRutina,"no");
  if (nombreRutina){
    const msg = {
      orden: ['addRT', nombreRutina],
      coordenadas: [] //luego agregar posicion aca coordenadas: [posicion]
    };
    console.log(" Enviando a /orden_web: agregar", msg);

    // Publica en ROS
    pubTopic('orden_web', msg);
  }  
}

//Envia Orden para traer puntos desde la libreria de punto (Refrescar)
export function cargarRutinasDB() {
  const msg = {
    orden: ['subirR_db'],
    coordenadas: []
  };

  console.log(" Enviando a /orden_web:", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

//////////////////////////    FIN Section1     /////////////////////////


//////////////////////////    Section2     ////////////////////////////


export function correrTAngular( coord:number[],descriptcion:number[]) {
  const msg = {
    descriocion:descriptcion,
    coordenadas: coord
  };

  console.log(" Enviando a /cord_ros: correr angulos", msg);

  // Publica en ROS
  pubTopic('cord_ros', msg);
}

export function correrTCartesiano( coord:number[]) {
  const msg = {
    data: coord
  };

  console.log(" Enviando a /cord_ros: correr cartesianos", msg);

  // Publica en ROS
  pubTopic('cord_ros', msg);
}

export function escucharPuntoDB(callback: (msg: any) => void) {
  subTopic("puntodb", callback);

  // ❌ Luego analizar si conviene cerrarlo o cuando hacerlo
}

export function escucharCordReal(callback: (msg: any) => void) {
  subTopic("pos_real", callback);

  // ❌ Luego analizar si conviene cerrarlo o cuando hacerlo
}

//////////////////////////    FIN Section2     /////////////////////////

export function agregarPuntoRutina(nombrePunto:string,plan:string, coord:number[]) {
  const msg = {
    orden: ['addPT', nombrePunto,plan],
    coordenadas: coord
  };

  console.log(" Enviando a /orden_web: agregando punto rutina", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

//////////////////////////    Section3     ////////////////////////////


export function resibirMsgRutina(callback: (msg: any) => void) {
  console.log("verificando");
  subTopic("puntoRutina", callback);
}

//Elimina Punto de Rutina
export function elimiarPuntoRutina(nombrePunto:string[], posiciones:number[]) {
  const msg = {
    orden: ['eliminarPunRut', ...nombrePunto],
    coordenadas: posiciones
  };

  console.log("del", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

//Refresca Rutina actual
export function refrescarRutina() {
  const msg = {
    orden: ['refresRut'],
    coordenadas:[]
  };  

  console.log("refrescar", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

//Corer Trayectoria guardada
export function runTrayectoria() {
  const msg = {
    orden: ['runTrayectoria'],
    coordenadas:[]
  };  

  console.log("runTrayectoria", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

export function editarPuntoRutina(nombrePunto:string,plan:string, coord:number[]) {
  const msg = {
    orden: ['editPR', nombrePunto,plan],
    coordenadas: coord
  };

  console.log(" Enviando a /orden_web: editando punto rutina", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

export function ejecutarRutina() {
  const msg = { data: true };

  console.log(" Correr rutina", msg);

  // Publica en ROS
  pubTopic('rutina', msg);
}

//Agregar una rutina(la rutina actual la guarda asignandole nombre)
export function agregarRutina(nombreRutina:string) {
  const msg = {
    orden: ['addRutina', nombreRutina],
    coordenadas:[]
  };  

  console.log("addRutina", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

//////////////////////////    General     ////////////////////////////

export function refreshCordAng(coord:number[]) {
  const msg = {
    orden: ['ang-cart'],
    coordenadas: coord
  };

  console.log(" Enviando a /orden_web: editando punto rutina", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

export function refreshCordCart(coord:number[]) {
  const msg = {
    orden: ['cart-ang'],
    coordenadas: coord
  };

  console.log(" Enviando a /orden_web: editando punto rutina", msg);

  // Publica en ROS
  pubTopic('orden_web', msg);
}

export function publicar_informe(informe: string) {
  const msg = { data: informe };

  console.log(" informar", msg);

  // Publica en ROS
  pubTopic('informe_web', msg);
}

//Resibe mensaje de informe
export function msgInforme(callback: (msg: any) => void) {
  subTopic("informe_web", callback);
  
}

export function msgEmergente(tipoMsg: string) {
  let mensaje: string = "";   // inicializado en vacío

  switch (tipoMsg) {    
    case 'errorR':
      mensaje = [
        "⚠ Rutina no encontrada en base de datos lo que infiere una desincronizacion con esta.",
          "🔄 Refresque la tabla para asegurarse de trabajar con los datos reales.",
      ].join("\n");
      break;

    case 'errorRR':
      mensaje = [
        "⚠ La rutina cargada contiene rutinas no existentes entre sus instrucciones.",
      ].join("\n");
      break;

    case 'errorP':
      mensaje = [
        "⚠ Punto no encontrado en base de datos lo que infiere una desincronizacion con esta.",
          "🔄 Refresque la tabla para asegurarse de trabajar con los datos reales.",
      ].join("\n");
      break;

    case 'delPR':
      // lógica si eliminás bien
      break;

    case 'ErrordelPR':
      mensaje = [
        "⚠ No se eliminó ninguna fila, algunas instrucciones no coinciden con la base de datos lo que infiere una desincronizacion con esta.",
        "🔄 Refresque la tabla para asegurarse de trabajar con los datos reales.",
      ].join("\n");
      break;

    default:
      console.warn("Acción no reconocida:", tipoMsg);
  }

  if (mensaje) {
    alert(mensaje);
  }
};
