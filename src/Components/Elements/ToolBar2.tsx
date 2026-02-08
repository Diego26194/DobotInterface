import { AppBar, Toolbar, Button, Box } from "@mui/material";
import { Link } from "react-router-dom";
import { inicializarPrograma, cerrarPrograma, modoControl, modoLectura } from "../../Services/Funciones";

interface ToolBarProps {
  LinkHome: string;
  onTogglePuntos: () => void;
  onToggleRutinas: () => void;
  modoActuar: boolean;
}

function ToolBar({
  LinkHome,
  onTogglePuntos,
  onToggleRutinas,
  modoActuar,
}: ToolBarProps) {
  return (
    <AppBar
      position="static"
      sx={{
        backgroundColor: modoActuar? "#116e7aff":"rgb(17, 122, 78)" ,
        height: "clamp(36px, 4vh, 56px)",
        minHeight: "clamp(36px, 4vh, 56px)",
        justifyContent: "center",
      }}
    >
      <Toolbar
        disableGutters
        sx={{
          height: "100%",
          minHeight: "100% !important",
          px: 2,
          display: "flex",
          alignItems: "center",
          gap: 1,
        }}
      >
        <Button title={'Lista de Puntos Guardados'} onClick={onTogglePuntos}  sx={{height: "90%", color: "#f1eaeaff" }}>
          B.Puntos
        </Button>

        <Button title={'Lista de Rutinas Guardadas'} onClick={onToggleRutinas} sx={{height: "90%", color: "#f1eaeaff" }}>
          B.Rutinas
        </Button>

        <Button title={'Inicializar Controlador'} onClick={inicializarPrograma}   sx={{height: "90%", color: "#f1eaeaff" }}>
          Inicializar Programa
        </Button>

        <Button title={'Cerrar Controlador'} onClick={cerrarPrograma}   sx={{height: "90%", color: "#f1eaeaff" }}>
          Cerrar Programa
        </Button>

        <Button title={'Modo Control'} onClick={modoControl}   sx={{height: "90%", color: "#f1eaeaff" }}>
          M Control
        </Button>

        <Button title={'Modo Lectura'} onClick={modoLectura}   sx={{height: "90%", color: "#f1eaeaff" }}>
          M Lectura
        </Button>
        
        <Button
          component={Link}
          to={LinkHome}
          sx={{height: "90%", color: "#f1eaeaff" }}
        >
          Emulador Robot
        </Button>
      </Toolbar>
    </AppBar>
  );
}

export default ToolBar;
