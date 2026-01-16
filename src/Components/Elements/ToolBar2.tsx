import { AppBar, Toolbar, Button, Box } from "@mui/material";
import { Link } from "react-router-dom";
import { inicializarPrograma, cerrarPrograma, modoControl, modoLectura } from "../../Services/Funciones";

interface ToolBarProps {
  LinkHome: string;
  onTogglePuntos: () => void;
  onToggleRutinas: () => void;
}

function ToolBar({
  LinkHome,
  onTogglePuntos,
  onToggleRutinas,
}: ToolBarProps) {
  return (
    <AppBar
      position="static"
      sx={{
        backgroundColor: "#116e7aff",
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
        <Button onClick={onTogglePuntos}  sx={{height: "90%", color: "#f1eaeaff" }}>
          B.Puntos
        </Button>

        <Button onClick={onToggleRutinas} sx={{height: "90%", color: "#f1eaeaff" }}>
          B.Rutinas
        </Button>

        <Button onClick={inicializarPrograma}   sx={{height: "90%", color: "#f1eaeaff" }}>
          Inicializar Programa
        </Button>

        <Button onClick={cerrarPrograma}   sx={{height: "90%", color: "#f1eaeaff" }}>
          Cerrar Programa
        </Button>

        <Button onClick={modoControl}   sx={{height: "90%", color: "#f1eaeaff" }}>
          M Control
        </Button>

        <Button onClick={modoLectura}   sx={{height: "90%", color: "#f1eaeaff" }}>
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
