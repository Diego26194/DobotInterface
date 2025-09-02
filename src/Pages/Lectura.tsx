import { useParams } from "react-router-dom";
import { Container, Typography, CircularProgress, List, Paper } from "@mui/material";


const Lectura = () => {
  const items = ['🍎', '🍌', '🍇'];



  
  function manejarClick() {
      alert('¡Hiciste clic!');
    }

    


  return (

    <Container>
      <ul>
        {items.map((fruta, index) => (
          <li key={index}>{fruta}</li>
        ))}
      </ul>
      
      

      <button onClick={manejarClick}>Clic</button>;

      <Typography variant="h5" gutterBottom>Comentarios putos</Typography>
      <List component={Paper} sx={{ bgcolor: "background.paper" }}>
      </List>
    </Container>
  );
};

export default Lectura;