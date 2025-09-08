import { MenuItem } from "@mui/material";
import EastIcon from '@mui/icons-material/East';

interface ButtonRutineAddProps {
  onClick: () => void;
  description?: string;
}

const ButtonRutineAdd = ({ onClick, description }: ButtonRutineAddProps) => {
  return (
    <MenuItem title={description} onClick={onClick}
      sx={{
      minHeight: "28px",  // baja la altura mínima
      padding: "2px 1px", // menos espacio interno
      fontSize: "0.8rem", // texto más chico (si hubiera texto)
      }}
    >
      <EastIcon fontSize="small" style={{ marginRight: 2 }}/> {/*fontSize="large"*/} 
    </MenuItem>
  );
};

export default ButtonRutineAdd;