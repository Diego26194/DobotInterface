import { AppBar, Toolbar, Button, Box } from "@mui/material";
import { Link } from "react-router-dom";
import { cargarPuntosDB, cargarRutinasDB } from "../../Services/Funciones";

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

  const abrirRutinas = () => {
      onToggleRutinas
      cargarRutinasDB
    };

  const abrirPuntos = () => {
      onTogglePuntos
      cargarPuntosDB
    };

  return (
    <AppBar
      position="static"
      sx={{
        backgroundColor: "rgba(205, 207, 214, 0.67)",
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
        <Button onClick={abrirPuntos}  variant="contained" sx={{ backgroundColor: "rgba(73, 202, 101, 0.67)", height: "90%" }}>
          B.Puntos
        </Button>

        <Button onClick={abrirRutinas} variant="contained" sx={{ backgroundColor: "rgba(73, 202, 101, 0.67)", height: "90%" }}>
          B.Rutinas
        </Button>

        <Button variant="contained" sx={{ backgroundColor: "rgba(73, 202, 101, 0.67)", height: "90%" }}>
          Inicializar Programa
        </Button>

        <Button variant="contained" sx={{ backgroundColor: "rgba(73, 202, 101, 0.67)", height: "90%" }}>
          Cerrar Programa
        </Button>

        <Button variant="contained" sx={{ backgroundColor: "rgba(73, 202, 101, 0.67)", height: "90%" }}>
          M Escritura
        </Button>

        <Button variant="contained" sx={{ backgroundColor: "rgba(73, 202, 101, 0.67)", height: "90%" }}>
          M Lectura
        </Button>
        
        <Button
          component={Link}
          to={LinkHome}
          variant="contained" 
          sx={{ backgroundColor: "rgba(73, 202, 101, 0.67)", height: "90%" }}
        >
          Emulador Robot
        </Button>
      </Toolbar>
    </AppBar>
  );
}

export default ToolBar;
