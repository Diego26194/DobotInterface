import React, { useState, useEffect, useRef, forwardRef, useImperativeHandle } from "react";
import Box from '@mui/material/Box';
import { DataGrid, GridColDef, GridRowId, GridRowSelectionModel } from '@mui/x-data-grid';


import { Select, MenuItem, TextField } from '@mui/material';
import { SelectChangeEvent, IconButton } from '@mui/material';

import EditIcon from "@mui/icons-material/Edit";
import DoneIcon from "@mui/icons-material/Done";
import PlayArrowIcon from '@mui/icons-material/PlayArrow';

import { resibirMsgRutina ,elimiarPuntoRutina, correrTAngular, msgEmergente, editarPuntoRutina } from "../Services/Funciones";
import InputPoisitive from "../Components/Elements/Inputs/InputPoisitive";

import InputGridCords from '../Components/Elements/Inputs/InputGridCords';


//COLUMNA Select

const opcionesTrayectoria = ['PTP', 'LIN', 'CIRC'];

const SelectTrayectoriaCell = (params: any) => {
  const { id, field, value, api } = params;

  const handleChange = (event: SelectChangeEvent) => {
    api.setEditCellValue({ id, field, value: event.target.value }, event);
    api.commitCellChange({ id, field });
    api.setCellMode(id, field, 'view');
  };

  return (
    <Select
      autoFocus
      value={value || ''}
      onChange={handleChange}
      fullWidth
      size="small"
      sx={{
        width: "100%",
        fontSize: "0.75rem",
        padding: "0px 1px",
        minHeight: "20px",
        maxHeight: "26px",
        textAlign: "center",
      }}
    >
      {opcionesTrayectoria.map((opcion) => (
        <MenuItem 
          key={opcion} 
          value={opcion} 
          sx={{
            width: "100%",
            fontSize: "0.75rem",
            padding: "0px 12px",
            minHeight: "24px",
            textTransform: "none",
            justifyContent: "center",
          }}>
          {opcion}
        </MenuItem>
      ))}
    </Select>
  );
};

interface NumericEditCellProps {
  id: GridRowId;
  field: string;
  value?: number;
  api: any;
}

const NumericEditCell: React.FC<NumericEditCellProps> = ({ id, field, value, api }) => {
  const handleChange = (newValue: number) => {
    api.setEditCellValue({ id, field, value: newValue });
  };

  return (
    <InputPoisitive
      value={value ?? 0}
      valMin={0}
      valMax={100}
      onChange={handleChange}
      variant="outlined"
      sx={{'& .MuiInputBase-root': {
          height: 20,           // Igual a tu rowHeight
          fontSize: '0.75rem',  // Tamaño legible
          padding: '0 4px',     // Pequeño padding horizontal
        },
        '& input': {
          height: '100%',       // Para que ocupe todo el contenedor
          padding: 0,           // Sin padding extra
          fontSize: '0.75rem',  // Mismo tamaño de font
        },
      }}
    />
  );
};

type RowType = {
  id: number; 
  posicion: number;           
  nombre: string;
  coordenadas: number[]; // 6 elementos
  escV: number ; 
  ratio: number ; 
  plan: string ;
  editable: boolean;
  wait: number; 
};


interface RutineTableProps {
  flagEliminarPRut: boolean;
  setFlagEliminarPRut: React.Dispatch<React.SetStateAction<boolean>>;
}

export type RutineTableRef = {
  updateWaitForSelectedRows: (valor: number) => void;
  validarRutina: () => boolean; 
};
const RutineTable = forwardRef<RutineTableRef,RutineTableProps> (({flagEliminarPRut, setFlagEliminarPRut}, ref) => {

//const RutineTable = () => {
  const [rowSelectionModel, setRowSelectionModel] =
    React.useState<GridRowSelectionModel>({ type: 'include', ids: new Set() });
  //const [rows, setRows] = useState<RowType[]>([]);
  
  const [rows, setRows] = useState([ { id: 1, posicion: 1 , nombre: 'Inicio', coordenadas: [0, 0, 0, 0, 0, 0], escV: 10, ratio:0, plan: 'PTP' , editable: false, wait: 0},
     { id: 2, posicion: 2 , nombre: 'Punto A', coordenadas: [10, 20, 30, 0, 0, 0], escV: 50, ratio:50, plan: 'LIN' , editable: false, wait: 3},
     { id: 3, posicion: 3, nombre: 'Ir a Torno', coordenadas: [15, 25, 35, 0, 10, 0], escV: 100, ratio:0, plan: 'Rutina' , editable: false, wait: 0 }, 
     { id: 4, posicion: 4 , nombre: 'Punto B', coordenadas: [10, 20, 30, 0, 0, 0], escV: 50, ratio:50, plan: 'LIN' , editable: false, wait: 3}, 
     { id: 5, posicion: 5 , nombre: 'T2', coordenadas: [10, 20, 30, 0, 0, 0], escV: 50, ratio:50, plan: 'Trayectoria' , editable: false, wait: 3}, 
    ]);
    
    //const [wite, setShoWite] = useState(0);

  useEffect(() => {
    if (flagEliminarPRut) {
      const idsArray = Array.from(rowSelectionModel.ids);
      const nombresArray = rows
      .filter((row) => idsArray.includes(row.id)) 
      .map((row) => row.nombre);

      const posicionesArray = rows
      .filter((row) => idsArray.includes(row.id))
      .map((row) => row.posicion);
      
      if (idsArray.length > 0) {
        elimiarPuntoRutina(nombresArray, posicionesArray);
        //eliminarInstruccionRutina(idsArray.map(Number),['delPR','Punto A','Ir a Torno']);
        eliminarInstruccionRutina(idsArray.map(Number));
        //msgEmergente('ErrordelPR');
      }

      setFlagEliminarPRut(false);
    }
  }, [flagEliminarPRut, rowSelectionModel, setFlagEliminarPRut]);

  useEffect(() => {
    resibirMsgRutina((msg) => {
      
      switch (msg.orden[0]) {
        case 'editPR':
          setRows((prev) =>
            prev.map((row) =>
              row.posicion === msg.coordenadas[9]
                ? {
                    ...row,
                    nombre: msg.orden[1],
                    plan: msg.orden[2],
                    coordenadas: msg.coordenadas.slice(0, 6),
                    escV: msg.coordenadas[6],
                    ratio: msg.coordenadas[7],
                    wait: msg.coordenadas[8] ?? row.wait, // por si lo mandás
                    editable: false,
                  }
                : row
            )
          );
          break;

        case 'vaciarTabla':
          setRows([]);
          break;
          
        case 'addT':
          console.log('Trayectoria agregada');
          console.log(msg);
          addRowRutina(msg.orden, msg.coordenadas);
          
          break;
          
        case 'addRT':
          console.log('rutina agregada');
          console.log(msg);
          addRowRutina(msg.orden, msg.coordenadas);
          
          break;


        case 'errorPunRut':
          //msgEmergente('ErrordelPR');
          break;

        case 'errorPunRut2':
          // ejemplo: marcar fila en error
          /*
          setRows((prev) =>
            prev.map((row) =>
              row.posicion === msg.coordenadas[9]
                ? { ...row, error: true }
                : row
            )
          );
          */
          break;
          
        case 'addP':
          addRowPunto(msg.orden, msg.coordenadas);
          break;

        case 'errorP':
          msgEmergente('errorP');
          break;

        case 'delPR':
          eliminarInstruccionRutina(msg.coordenadas);
          break;

        case 'ErrordelPR':
          msgEmergente('ErrordelPR');
          break;

        case 'errorRR':
          msgEmergente('errorRR');
          break;

        case 'errorR':
          msgEmergente('errorR');
          break;

        default:
          console.warn("Acción no reconocida:", msg.orden[0]);
      }
    });
  }, []);

  useImperativeHandle(ref, () => ({
    updateWaitForSelectedRows,
    validarRutina: () => {
      const algunEditable = rows.some((row) => row.editable === true);
      return !algunEditable; 
    },
  }));

  {/* 
  useImperativeHandle(ref, () => ({
    updateWaitForSelectedRows,
  }));
  */}

  const updateWaitForSelectedRows = (newWait: number) => {
    console.log(newWait);
    setRows((prevRows) =>
      prevRows.map((row) =>
        rowSelectionModel.ids.has(row.id) ? { ...row, wait: newWait } : row
      )
    );
    console.log(rows);
  };
  
  const addRowPunto = (orden: string[], coordenadas: number[]) => { //orden [id,noombre,plan] coordenadas[a1,a2,a3,a4,a5,a6,vel, ratio]
    setRows((prev) => {
      const newRow: RowType = {
        id: Date.now() + Math.floor(Math.random() * 1000), //Date.now(),      
        posicion: coordenadas[9], //prev.length + 1,         
        nombre: orden[1],
        plan: orden[2],
        coordenadas: coordenadas.slice(0, 6),
        escV: coordenadas[6], 
        ratio: coordenadas[7], 
        editable: false,
        wait: coordenadas[8],          
      };
      return [...prev, newRow];
    });
  };  
  
  const addRowRutina = (orden: string[], coordenadas: number[]) => { //orden [id,noombre,plan] coordenadas[a1,a2,a3,a4,a5,a6,vel, ratio]
    setRows((prev) => {
      const newRow: RowType = {
        id: Date.now() + Math.floor(Math.random() * 1000), //Date.now(),      
        posicion: coordenadas[1], //prev.length + 1,         
        nombre: orden[1],
        plan:  orden[2],
        coordenadas: [],
        escV: 0, 
        ratio: 0, 
        editable: false,
        wait: coordenadas[0],         
      };
      return [...prev, newRow];
    });
  }; 
   
  const eliminarInstruccionRutina = (posiciones: number[]) => {
    setRows((prevRows) => {
      // 1. Filtrar las filas que NO están en posiciones a eliminar
      const filtradas = prevRows.filter(row => !posiciones.includes(row.posicion));

      // 2. Recalcular posiciones
      return filtradas.map((row, index) => ({
        ...row,
        posicion: index + 1
      }));
    });
  };
{/* 
  const FalloEliminarIRutina=(posiciones: number[])=> {
    const mensaje = [
      "              ⚠ No se elimino nunguna fila:",
      " Las siguientes instrucciones no se encuentran en la rutina o no coinciden con la base de datos:",
      ...posiciones.map(
        (item) =>
          `- Posición ${item}`,
      ),
      "",
      "🔄 Refresque la tabla para asegurarse de trabajar con los datos reales.",
    ].join("\n");

    alert(mensaje);
  }
  */}
  {/* 
  const eliminarInstruccionRutina = (posiciones: number[], orden: string[]) => {
  setRows((prevRows: RowType[]) => {
    // Relacionar posiciones con nombres esperados
    const objetivos = posiciones.map((pos, i) => ({
      pos,
      nombreEsperado: orden[i + 1],
    }));

    const noCoinciden: { pos: number; esperado: string; encontrado?: string }[] = [];

    // Filtrar filas
    const filtradas = prevRows.filter((row) => {
      const objetivo = objetivos.find((o) => o.pos === row.posicion);
      if (!objetivo) return true; // no estaba en la lista → se queda

      if (row.nombre === objetivo.nombreEsperado) {
        return false; // ✅ coincide, se elimina
      } else {
        // ❌ no coincide → lo guardamos como error
        noCoinciden.push({
          pos: row.posicion,
          esperado: objetivo.nombreEsperado,
          encontrado: row.nombre,
        });
        return true; // no lo eliminamos
      }
    });

    // Mostrar popup si hubo inconsistencias
    if (noCoinciden.length > 0) {
      const mensaje = [
        "⚠ Los siguientes puntos no se encuentran en la rutina o no coinciden con la base de datos:",
        ...noCoinciden.map(
          (item) =>
            `- Posición ${item.pos}: esperado "${item.esperado}" pero encontrado "${item.encontrado ?? "ninguno"}"`
        ),
        "",
        "🔄 Refresque la tabla para asegurarse de trabajar con los datos reales.",
      ].join("\n");

      alert(mensaje);
    }    

    // Recalcular posiciones
    return filtradas.map((row, index) => ({
      ...row,
      posicion: index + 1,
    }));
  });
};
*/}

  
  const handleEdit = (id: number) => {
    setRows((prev) =>
      prev.map((row) =>
        row.id === id ? { ...row, editable: true } : row
      )
    );
   // const pruebaID = mandarIDSeleccionados();
    //console.log(" los numeros ganadores son", pruebaID);
    //console.log(" los numeros ganadores son", rowSelectionModel);
  };

  const handleConfirm = (id: number) => {
    setRows((prev) => {
      const row = prev.find(r => r.id === id);
      if (!row) return prev;

      const coord = [
        ...row.coordenadas, // los 6 ángulos
        row.escV,
        row.ratio,
        row.wait,
        row.posicion,
      ];

      // Publicar en ROS, no tocar filas todavía
      editarPuntoRutina(row.nombre, row.plan, coord);

      return prev;
    });
  };

  const handlePlayRow = (rowId: number) => {
    // Buscamos la fila correspondiente en el estado `rows`
    const row = rows.find(r => r.id === rowId);
    if (row) {
      if (row.plan="Rutina"){
        console.log("No es un punto,por el momento no se puede correr la rutina: ", row.nombre);
      }
      else{
        console.log("Coordenadas:", row.coordenadas);
        alert(`Coordenadas: [${row.coordenadas.join(', ')}]`);

        switch (row.plan) {
                
          case "PTP":
            correrTAngular([row.posicion],[2,1,row.escV]);
            break;          
          
          case "LIN":
            correrTAngular([row.posicion],[2,2,row.escV]);
            break;

          case "CIRC":
            correrTAngular([row.posicion],[2,3,row.escV]);
            break;

          default:
            console.warn("Acción no reconocida:", row.plan);
        }
      }
    }
  };

  const columns: GridColDef[] = [
    { field: 'posicion', headerName: 'Pos.', width: 40 },
    {
      field: 'nombre',
      headerName: 'Nombre',
      width: 100,
      editable: true,
    },
  {
      field: 'coordenadas',
      headerName: 'Coordenadas',
      width: 360,
      editable: true,  
      renderEditCell: ({ id, field, value, api }) => {
        const handleChange = (index: number) => (e: React.ChangeEvent<HTMLInputElement>) => {
          const newCoords = [...value];
          newCoords[index] = parseFloat(e.target.value) || 0;
          api.setEditCellValue({ id, field, value: newCoords });
        };

        return (
          <Box sx={{ display: 'flex', gap: 0.7 }}>
            {value.map((coord: number, i: number) => (
              <InputGridCords
                key={i}
                value={coord}
                onChange={handleChange(i)}
              />
            ))}
          </Box>
        );
      },
      renderCell: (params) => (params.row.plan !== "Rutina" && params.row.plan !== "Trayectoria") ? 
        (<span>{params.value?.join(' , ')}</span>
      ) : null
    },
    {
      field: 'escV',
      headerName: 'Esc.V',
      type: 'number',
      width: 50,
      editable: true,
      renderEditCell: (params) => <NumericEditCell {...params} />,
      renderCell: (params) =>(params.row.plan !== "Rutina" && params.row.plan !== "Trayectoria" ) ? 
      params.row.escV : null
    },
  /*  {
      field: 'escA',
      headerName: 'Esc.A',
      type: 'number',
      width: 80,
      editable: true,
    },
    {
      field: 'radio',
      headerName: 'Radio',
      type: 'number',
      width: 80,
      editable: true,
    },
    */
    {
      field: 'ratio',
      headerName: 'Ratio',
      type: 'number',
      width: 50,
      editable: true,
      renderEditCell: (params) => <NumericEditCell {...params} />,
      renderCell: (params) =>(params.row.plan !== "Rutina" && params.row.plan !== "Trayectoria" ) ? 
      params.row.ratio : null
    },
    {
      field: 'plan',
      headerName: 'Plan',
      width: 80,
      editable: true,
      renderEditCell: (params) => <SelectTrayectoriaCell {...params} />,
      
    },
    {
      field: "actions",
      headerName: "Edit",
      width: 40,
      renderCell: (params) => {

        if ((params.row.plan == "Rutina" || params.row.plan == "Trayectoria" )) return null;

        const rowEditable = params.row.editable;

        return rowEditable ? (
          <IconButton
            onClick={() => handleConfirm(params.row.id)}
            color="success"       
            sx={{ 
              height: '25%',              
              minHeight: '2px',
              "&:focus": { outline: "none" },   // saca el borde de focus
              "&:focus-visible": { outline: "none" },
            }}
          >
            <DoneIcon fontSize="small"/>
          </IconButton>
        ) : (
          <IconButton
            onClick={() => handleEdit(params.row.id)}
            color="primary"
            sx={{ 
              height: '25%',              
              minHeight: '2px',
              "&:focus": { outline: "none" },   // saca el borde de focus
              "&:focus-visible": { outline: "none" },
            }}
          >
            <EditIcon fontSize="small"/>
          </IconButton>
        );
      },
    },
    {
      field: "correr",
      headerName: "Correr",
      width: 40,
       renderCell: (params) =>
        !params.row.editable ? (
          <IconButton
            onClick={() => handlePlayRow(params.row.id)}
            color="primary"
            sx={{
              height: '25%',
              minHeight: '2px',
              "&:focus": { outline: "none" },
              "&:focus-visible": { outline: "none" },
            }}
          >
            <PlayArrowIcon fontSize="small" />
          </IconButton>
        ) : null,
    },
    {
      field: "wait",
      headerName: "Wait",
      type: 'number',
      width: 40,
      editable: true,
      renderEditCell: (params) => <NumericEditCell {...params} />,
      renderCell: (params) =>params.row.wait !== 0 ? params.row.wait : null
    }
  ];

  return (
    <Box sx={{ height: '100%', width: '100%'
      
    }}>
      <DataGrid
        rows={rows}
        columns={columns}
        checkboxSelection
        disableRowSelectionOnClick
        rowHeight={20}
        hideFooter
        isCellEditable={(params) => params.row.editable === true}
        processRowUpdate={(newRow, oldRow) => {
          // Acá actualizás el estado con los valores editados
          setRows((prev) =>
            prev.map((row) => (row.id === newRow.id ? { ...newRow } : row))
          );
          return newRow; // importante que retorne el nuevo row
        }}

        //rowSelectionModel={selectionModel}
        //onRowSelectionModelChange={(newModel) => {setSelectionModel(newModel);}}
        onRowSelectionModelChange={(newRowSelectionModel) => {
          setRowSelectionModel(newRowSelectionModel);
        }}
        rowSelectionModel={rowSelectionModel}

        sx={{
          border: '1px solid #000',
          maxHeight: "100%",
          '& .MuiDataGrid-cell': {
            fontSize: '0.75rem', // Tamaño de letra
            py: 0.0, // padding vertical reducido (altura de fila)
            
            maxHeight: 20,
            minHeight: 20,
          },
          '& .MuiDataGrid-columnHeaders': {
            minHeight: 32,   // Altura mínima total del encabezado
            maxHeight: 32,
          },
          '& .MuiDataGrid-columnHeader': {
            fontSize: '0.75rem',
            py: 0,
            maxHeight: 30,
            minHeight: 30,
          },
          '& .MuiDataGrid-row': {
            maxHeight: 30,
            minHeight: 30,
          },
        }}
      />
    </Box>
  );
});

export default RutineTable;
