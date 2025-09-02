import { AppBar, Toolbar, Button } from "@mui/material";
import { Link } from "react-router-dom";

interface ToolBarProps {
  LinkHome: string;
}

function ToolBar({ LinkHome }: ToolBarProps) {
  return (
    <AppBar position="fixed" sx={{ backgroundColor: "rgba(185, 194, 158, 0.7)", width: "100%", height: "4%" }}>
      <Toolbar>
          <Button sx={{ color: "black", fontSize: "10px", height: "90%" }}>
            Inicio
          </Button>
      </Toolbar>
    </AppBar>
  );
}

export default ToolBar;
