import Escritura from "./Pages/Escritura";
import Lectura from "./Pages/Lectura";
import Arduino from "./Pages/arduino";
import Cinematica from "./Pages/Cinematic";
import ToolBar from "./Components/Elements/ToolBar";

import RutineTable from "./Pages/RutineTable";
import Prueba2 from "./Pages/prueba2"

import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import { Box, useTheme } from "@mui/material";

import { ThemeProvider, createTheme } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';

const lightTheme = createTheme({
  palette: {
    mode: 'light',
  },
});

function App() {
  return (
    <ThemeProvider theme={lightTheme}>
      <CssBaseline />
      <Router >
        {/*<ToolBar LinkHome="/lectura" /> 

         Espaciador para el AppBar fijo */}
        {/*<Box sx={(theme) => ({ ...theme.mixins.toolbar })} />
*/}
        {/* Contenido principal */}
        <Box >
          <Routes>
            <Route path="/" element={<Escritura />} />
            <Route path="/2" element={<Lectura />} />
            <Route path="/p2" element={<Prueba2 />} />
            <Route path="/arduino" element={<Arduino />} />
            <Route path="/c" element={<Cinematica />} />
          </Routes>
        </Box>
      </Router>
    </ThemeProvider>
  );
}

export default App;
