import { MenuItem } from "@mui/material";
import SaveIcon from "@mui/icons-material/Save";

interface ButtonSaveProps {
  onClick: () => void;
  description?: string;
}

const ButtonSave = ({ onClick, description }: ButtonSaveProps) => {
  return (
    <MenuItem title={description} onClick={onClick}
      sx={{
        minHeight: "28px",  // baja la altura mínima
        padding: "2px 6px", // menos espacio interno
        fontSize: "0.8rem", // texto más chico (si hubiera texto)
      }}
    >
      <SaveIcon fontSize="small" style={{ marginRight: 2 }} />
    </MenuItem>
  );
};

export default ButtonSave;